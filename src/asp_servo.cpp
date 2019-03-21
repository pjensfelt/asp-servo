/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/
#include "asp_servo_api/constants.h"
#include "asp_servo_api/asp_servo.h"
#include "asp_servo_api/servo.h"
#include "asp_servo_api/ServoInfo.hpp"
#include <soem/ethercat.h>
#include <tinyxml2.h>
#include <stdexcept>
#include <sys/time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <exception>

namespace asp {


    // Constructor - read settings, create the servo objects and initialize the global servoInfo_map
    ServoCollection::ServoCollection(std::string fullfilename) {

        tinyxml2::XMLDocument doc;
        const char * FULLFILENAME = fullfilename.c_str();
        tinyxml2::XMLError eResult = doc.LoadFile(FULLFILENAME);
        std::cout << "XML read result (0=no error): " << eResult << std::endl;

        tinyxml2::XMLElement *pRoot = doc.FirstChildElement("SystemConfiguration");

        tinyxml2::XMLElement *pEth = pRoot->FirstChildElement("EthernetPort");
        ethernetport_ = pEth->GetText();

        tinyxml2::XMLElement *pCyc = pRoot->FirstChildElement("CycleTime_ms");
        cycletime_ms_ = std::stoi(pCyc->GetText());


        tinyxml2::XMLElement *pServoCollection = pRoot->FirstChildElement("ServoCollection");
        tinyxml2::XMLElement *pServo = pServoCollection->FirstChildElement();
        while (pServo != NULL) {
            Servo* s = Servo::deserializeXML(pServo);
            servos_.insert(std::pair<std::string,Servo*>(s->get_name(),s));
            servo_by_pos_.insert(std::pair<int,Servo*>(s->get_position(),s));
            pServo = pServo->NextSiblingElement();
        }

        currentstate_ = EthercatStates::Init;
		initServoInfo();
		pthread_mutex_init(&stopallMx, NULL);
        std::cout << "Initialized ServoCollection from " << FULLFILENAME << std::endl;
    }

    // Destructor
    ServoCollection::~ServoCollection() {
        for (auto& kvp:servos_) {
            delete(kvp.second);
        }

		pthread_mutex_destroy(&stopallMx);
    }

	// Initializes the servo info structures. #Todo PROVIDE XML.
	void ServoCollection::initServoInfo(){
		// XML config file...
	 	// JOINT LIMITS
		servoInfo_["s1"] = asp::ServoInfo("s1", 1, "Z", 5044000.00, "m/s",   (int)( 0.4*5044000.00),  (int)(1.0*5044000.0)); 
		servoInfo_["s2"] = asp::ServoInfo("s2", 2, "X", 3991800.00, "m/s",   (int)( 0.1*3991800.00),  (int)(1.9*3991800.0)); 
		servoInfo_["s3"] = asp::ServoInfo("s3", 3, "Y", 5099400.00, "m/s",   (int)( 0.0*5099400.00),  (int)(0.60*5099400.0)); 
		servoInfo_["s4"] = asp::ServoInfo("s4", 4, "B", 1877468.10, "rad/s", (int)(-(0.8)*M_PI/2*1877468.0),  (int)(+(0.8)*M_PI/2*1877468.0)); 
		servoInfo_["s5"] = asp::ServoInfo("s5", 5, "A", 1564575.85, "rad/s", (int)0,  (int)(M_PI*1564575.85)); 
	}

	void ServoCollection::switchOnServo()
	{
		connect();
		require_ecat_state(asp::EthercatStates::Operational);
		require_servo_state(asp::ServoStates::OperationEnabled);
		
		// Update the servo state to not-stopped
		pthread_mutex_lock(&stopallMx);
		stopped = false;
		pthread_mutex_unlock(&stopallMx);
	}
	
	// Converts a velocity command expressed as m/s or rad/s into a proper command for the specified servo, and sends it.
	void ServoCollection::sendVelCmdSI(std::string servoName, double cmdSI)
	{
		/*If we're shutting down everything for some reason, return false*/
		/*if(stopped){
			throw "Cannot send command to servo motors (probably stopping)\n";
		}*/
    	std::map<std::string, asp::ServoInfo>::iterator sIt;
		sIt = servoInfo_.find(servoName);
		if(sIt == servoInfo_.end()){ // Should never happen
	        std::cerr << "Programming error, unknown servo name " << servoName << std::endl;
			stopAll();
		}
		write(servoName,"Velocity", sIt->second.toTicks(cmdSI)); 
	}

    // Setup communication, start cyclic async loop
    bool ServoCollection::connect() {

        std::vector<char> ethernetportvec(ethernetport_.c_str(), ethernetport_.c_str() + ethernetport_.size() + 1u);
        char* ethernetportchar = &ethernetportvec[0];

        if (ec_init(ethernetportchar)){
            std::cout << "Ethernetport " << ethernetport_ << " initialized!" << std::endl;
            
            if ( ec_config_init(FALSE) > 0 ) {
                std::cout << ec_slavecount << " slave(s) found and configured." << std::endl;
            }
            else {
                std::runtime_error("No slaves were found.");
            }
        }

        is_connected_ = true;

        // Start async thread
        async_future_ = std::async(std::launch::async, &ServoCollection::ethercat_loop, this);

        return true;
    }

    // Stop communication, exit async thread, and close socket
    bool ServoCollection::disconnect() {
        require_ecat_state(EthercatStates::PreOperational);
        is_connected_ = false;
        async_future_.get();
        ec_close();        
    }

	// Emergency stop and disconnect.
	void ServoCollection::stopAll(){
		
		pthread_mutex_lock(&stopallMx);
		if(!stopped){

			std::cerr << "Stopping everything" << std::endl;
			// Require that everyone goes into QUICK_STOP state
			require_servo_state(asp::ServoStates::QuickStopActive);

			// Shut everything down, disconnect
			require_servo_state(asp::ServoStates::SwitchOnDisabled);
			set_verbose(false);  
			disconnect();
			std::cerr << "Stopped everything" << std::endl;
			stopped = true;
		}
		pthread_mutex_unlock(&stopallMx);

		// Print final positions
		for (auto kvp: servoInfo_) {
		    int pos = servos_[kvp.first]->read_INT32("Position");
	        std::cout << "Position " << pos << " limits " << kvp.second.getLlimTicks() << " " << kvp.second.getUlimTicks();
		}
		return;
	}
	
	// Returns true if we're stopping 
	bool ServoCollection::isStopped(){
		bool isStopped;
		pthread_mutex_lock(&stopallMx);
		isStopped = stopped;
		pthread_mutex_unlock(&stopallMx);
		return isStopped;
	}

    // Move from current state to the required state. see Fig 2-2
    // This may include going through other states
    bool ServoCollection::require_ecat_state(EthercatStates requiredstate) {

        if (requiredstate == currentstate_) {
            return true;
        }

        // Transistion matrix gives the next state given the current state and the desired state
        std::vector<std::vector<EthercatStates>> transition_matrix {
            {EthercatStates::Init,           EthercatStates::PreOperational, EthercatStates::PreOperational, EthercatStates::PreOperational},
            {EthercatStates::Init,           EthercatStates::PreOperational, EthercatStates::SafeOperational,EthercatStates::SafeOperational},
            {EthercatStates::PreOperational, EthercatStates::PreOperational, EthercatStates::SafeOperational,EthercatStates::Operational},
            {EthercatStates::SafeOperational,EthercatStates::SafeOperational,EthercatStates::SafeOperational,EthercatStates::Operational}
        };

        EthercatStates nextstate = transition_matrix[currentstate_][requiredstate];

        if (currentstate_ == EthercatStates::Init && nextstate == EthercatStates::PreOperational) {
            std::cout << "Changing EtherCat state from Init to Pre-Op" << std::endl;
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::PreOperational && nextstate == EthercatStates::SafeOperational) {
            std::cout << "Changing EtherCat state from Pre-Op to Safe-Op" << std::endl;

            if (!has_done_SDO_and_PDO_configuration_) {

                std::cout << "SDO setup:" << std::endl;

                // Send objects and values to slaves according to StartupParameters in the XML file
                for (auto& kvp: servos_) {
                    Servo * pservo = kvp.second;
                    pservo->initialize_SDO_settings();
                }

                // Setup PDO according to the PDOmapping in the XML file
                for (auto& kvp : servos_) {
                    Servo* pservo = kvp.second;
                    pservo->do_PDO_mapping();
                    pservo->SDOread_INT32(1,0x1C32,2);
                }

                ec_config_map(&IOmap_);
                ec_configdc();
                has_done_SDO_and_PDO_configuration_ = true;

            }

            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
            PDO_input_enable_ = true;

        }
        else if (currentstate_ == EthercatStates::SafeOperational && nextstate == EthercatStates::Operational){
            std::cout << "Changing EtherCat state from Safe-Op to Operational" << std::endl;
            for (int slave_index = 1; slave_index <= ec_slavecount; slave_index++) {
               ec_slave[slave_index].state = EC_STATE_OPERATIONAL;
               ec_writestate(slave_index);
            }
            int state_res = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            std::cout << "Statecheck returned (8=operational): " << state_res << std::endl;
            int64_t sync0time_ns = cycletime_ms_ * 1000;
            ec_dcsync0(1, TRUE, sync0time_ns, 0); // SYNC0 on slave 1
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
               std::cout << "Operational state reached for all slaves." << std::endl;
               PDO_output_enable_ = true;
            }
            else {
                std::runtime_error("All slaves could not reach operational state");
            }
        }
        else if (currentstate_ == EthercatStates::Operational && nextstate == EthercatStates::SafeOperational) {
            std::cout << "Changing EtherCat state from Operational to Safe-Op" << std::endl;
            PDO_output_enable_ = false;  
            int64_t sync0time_ns = cycletime_ms_ * 1000;
            ec_dcsync0(1, FALSE, sync0time_ns, 0); // SYNC0 off  
            for (int slave_index = 1; slave_index <= ec_slavecount; slave_index++) {
               ec_slave[slave_index].state = EC_STATE_SAFE_OP;
               ec_writestate(slave_index);
            }
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::SafeOperational && nextstate == EthercatStates::PreOperational) {
            std::cout << "Changing EtherCat state from Safe-Op to Pre-Op" << std::endl;
            PDO_input_enable_ = false;            
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
        }
        else if (currentstate_ == EthercatStates::PreOperational && nextstate == EthercatStates::Init) {
            std::cout << "Changing EtherCat state from Pre-Op to Init" << std::endl;
            ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);
        } 

        currentstate_ = nextstate;

        return require_ecat_state(requiredstate);
    }

	bool ServoCollection::require_servo_state(ServoStates requiredstate) {

        std::map<std::string,ServoStates> intermediatestates;
        for (auto& kvp:servos_) {
            ServoStates currentstate = kvp.second->get_servo_state();
            ServoStates nextstate = kvp.second->require_servo_state(currentstate, requiredstate);
            intermediatestates.insert(std::pair<std::string,ServoStates>(kvp.first,nextstate));
        }

        int maxiter = 40;
        int iter = 0;
        bool aok;
        while (iter < maxiter) {
            aok = true;
            for (auto& kvp:servos_) {
                Servo* s = kvp.second;
                ServoStates currentstate = s->get_servo_state();
                if (currentstate != requiredstate) {
                    aok = false;
                    if (currentstate == intermediatestates[kvp.first]) {
                       ServoStates nextstate = s->require_servo_state(currentstate, requiredstate);
                       intermediatestates[kvp.first] = nextstate;
                    }
                }
            }
            if (aok) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            iter++;
        }

        if (!aok) {
            std::runtime_error("Not all servos reached the required state in 2s");
        }

        return true;

    }

    // Adding the cycle time to the time spec
    #define NSEC_PER_SEC 1000000000
    void add_timespec(struct timespec *ts, int64 addtime)
    {
       int64 sec, nsec;
       nsec = addtime % NSEC_PER_SEC;
       sec = (addtime - nsec) / NSEC_PER_SEC;
       ts->tv_sec += sec;
       ts->tv_nsec += nsec;
       if ( ts->tv_nsec > NSEC_PER_SEC )
       {
          nsec = ts->tv_nsec % NSEC_PER_SEC;
          ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
          ts->tv_nsec = nsec;
       }
    }

    // PI control to get linux time synced to DC time
    void ec_sync(int64 reftime, int64 cycletime_ns , int64 *offsettime_ns)
    {
       int integral = 0;
       int64 delta;
       /* set linux sync point 50us later than DC sync, just as example */
       //delta = (reftime - 50000) % cycletime_ns;
       // Not sure what to use here. Zero works as well /UE
       delta = (reftime) % cycletime_ns;
       if(delta > (cycletime_ns /2)) { 
           delta = delta - cycletime_ns; 
       }
       else if(delta>0) { 
           integral++; 
       }
       else if(delta<0) { 
           integral--; 
       }
       *offsettime_ns = -(delta / 100) - (integral /20);
    }

	void ServoCollection::checkInWorkspace()
	{
		int pos;
		double x, y, theta;
		double P1_p[2], P2_p[2];
		x = servoInfo_["s2"].SIfromTicks(servos_["s2"]->read_INT32("Position"));
		y = servoInfo_["s3"].SIfromTicks(servos_["s3"]->read_INT32("Position"));
		theta = servoInfo_["s4"].SIfromTicks(servos_["s4"]->read_INT32("Position"));
		
		P1_p[0] = P1[0]*cos(theta) - P1[1]*sin(theta) + X_OFFSET + x;
		P1_p[1] = P1[0]*sin(theta) + P1[1]*cos(theta) + Y_OFFSET + y;
		P2_p[0] = P2[0]*cos(theta) - P2[1]*sin(theta) + X_OFFSET + x;
		P2_p[1] = P2[0]*sin(theta) + P2[1]*cos(theta) + Y_OFFSET + y;
		
		if(    P1_p[0]< X_LIM[0]+TOLERANCE || P1_p[0]> X_LIM[1] - TOLERANCE   /*P1.x is outbounds*/
			|| P2_p[0]< X_LIM[0]+TOLERANCE || P2_p[0]> X_LIM[1] - TOLERANCE   /*P2.x is outbounds*/
			|| P1_p[1]< Y_LIM[0]+TOLERANCE || P1_p[1]> Y_LIM[1] - TOLERANCE   /*P1.y is outbounds*/
			|| P2_p[1]< Y_LIM[0]+TOLERANCE || P2_p[1]> Y_LIM[1] - TOLERANCE){ /*P2.y is outbounds*/
				stopAll();
				std::cerr << "Exiting safe space" << std::endl;
		}
		
		for (auto kvp: servoInfo_) {
		    pos = servos_[kvp.first]->read_INT32("Position");
		    if(!kvp.second.inLimitsTicks(pos)){
		        std::cout << "Stopping. Servo " << kvp.first << " not in limits" << std::endl;
		        std::cout << "Position " << pos << " limits " << kvp.second.getLlimTicks() << " " << kvp.second.getUlimTicks();
		        stopAll();
		        break;
		    }
			kvp.second.setPositionTicks(pos); // For logging when exiting
		}
	}

    // Asynchrounous loop
    void ServoCollection::ethercat_loop() {

        uint16_t control_word = 0;
        struct timespec tspec;
        struct timespec tspecdummy;
        struct timespec tcheck;
        struct timespec tcheck_old;
        int64_t cycletime_ns = cycletime_ms_ * 1000;
        int64_t offset_ns = 0;
        clock_gettime(CLOCK_MONOTONIC, &tspec);

        while (is_connected_) {

            // Basically adding the cycletime to the timespec
            add_timespec(&tspec, cycletime_ns + offset_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tspec, &tspecdummy);

            // For logging only
            clock_gettime(CLOCK_MONOTONIC, &tcheck);
            long deltat_us = (tcheck.tv_nsec - tcheck_old.tv_nsec)/1000;
            if (deltat_us < 0) {
                deltat_us += 1000000;
            }
            tcheck_old = tcheck;

            // Recieve data - possible in SAFE_OP and OPERATIONAL
            if (PDO_input_enable_) {
                ec_receive_processdata(EC_TIMEOUTRET);
            }

            // Syncing to DC
            ec_sync(ec_DCtime, cycletime_ns, &offset_ns);

            // Sending data - possible in OPERATIONAL only
            if (PDO_output_enable_) {

                for (auto& kvp : servos_) {  
                    int position = kvp.second->get_position();                  
                    kvp.second->update_IOmap(ec_slave[position].outputs, ec_slave[position].inputs);
                }
                
                ec_send_processdata();

                if (logging_enabled_) {

                    for (int i=1; i<=servo_by_pos_.size(); i++) {
                        std::cout << "#" << i << ":";
                        std::string logstr = servo_by_pos_[i]->read_logstring();
                        std::cout << logstr << " ";
                    }

                    std::cout << "dt:" << deltat_us << "us" << std::endl;
                }

				// Check if in workspace/ within joint limits			
				checkInWorkspace();
            }
        }
    }

    std::ostream& operator<<(std::ostream& strm, const ServoCollection& sc) {
        strm << "------------------- ServoCollection ---------------------" << std::endl;
        strm << "EthernetPort  : " << sc.ethernetport_ << std::endl;
        strm << "Cycletime (ms): " << sc.cycletime_ms_ << std::endl;
        for (auto& s:sc.servos_) {
            strm << *s.second << std::endl ;
        }
        strm << "---------------------------------------------------------" << std::endl;
        return strm;
    }

}
