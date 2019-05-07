/*
Code written by Urban Eriksson, urbane@kth.se

The implementation follows the HIWIN manual:

HIWIN CoE Drive User Guide V1.1.pdf

which can be found in the documents folder of the repository.
*/

#pragma once
#include "asp_servo_api/servo.h"
#include <tinyxml2.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <future>
#include <unistd.h>
namespace asp {

    // The states of the EtherCat state machine, see 2.3
    enum EthercatStates {
        Init,
        PreOperational,
        SafeOperational,
        Operational
    };
    /**
    * The ServoCollection class is implemented as a Singleton, to avoid any
    * conflicts whenever more than one thread would need to access it.
    */
    class ServoCollection {
        public:
        static ServoCollection& getInstance(const std::string fullfilename){
          static ServoCollection instance(fullfilename);
                                           // Guaranteed to be destroyed.
                                           // Instantiated on first use.
                                           // C++11 thread safe.
          std::cout << "getInstance()" << std::endl;
          return instance;
        }

        private:
          ServoCollection(const std::string fullfilename);
        public:
          ServoCollection(ServoCollection const&) = delete;
          void operator=(ServoCollection const &) = delete;

        friend std::ostream& operator<<(std::ostream& strm, const ServoCollection& sc);
        ~ServoCollection();
        // API methods
        bool connect();
        bool disconnect();

		/**
		* Switches on the servo motors
		*/
		void switchOnServo();
        bool require_ecat_state(EthercatStates newstate);
        bool require_servo_state(ServoStates newstate);
        void set_verbose(bool on) {logging_enabled_ = on;};
		/**
		* Stops all the servo motors by requiring them to go in the QuickStopActive state.
		* Turns them off and closes the connection.
		*/
		void stopAll();

        // Read actual values from the servos
		double read_SI(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_SI(entity_name);};
        uint16_t read_UINT16(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_UINT16(entity_name);};
        int16_t read_INT16(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_INT16(entity_name);};
        int read_INT32(std::string servo_name, std::string entity_name) {return servos_[servo_name]->read_INT32(entity_name);};

        // Write target values to servos
		void write_SI(std::string servo_name, std::string entity_name, double value) {servos_[servo_name]->write_SI(entity_name,value);}
        void write(std::string servo_name, std::string entity_name, uint16_t value) {servos_[servo_name]->write(entity_name,value);};
        void write(std::string servo_name, std::string entity_name, int16_t value) {servos_[servo_name]->write(entity_name,value);};
        void write(std::string servo_name, std::string entity_name, int value) {servos_[servo_name]->write(entity_name,value);};

        // Read/Write CN6 I/O
        uint16_t read_CN6_inputs(std::string servo_name){return servos_[servo_name]->read_CN6_inputs();}
        void enable_CN6_output(std::string servo_name){servos_[servo_name]->enable_CN6_output();};
        void write_CN6_outputs(std::string servo_name, uint8_t value){servos_[servo_name]->write_CN6_outputs(value);}

        // Need not be used except for debug purposes
        Servo* get_servo(std::string name){return servos_[name];};
        void set_servo(std::string name, int value) {};

        private:

        // The async loop outputting data to the servos.
        void ethercat_loop();
		// Checks if the "critical points" of the arm (2 points at the tip) are into the joint space.
		void checkInWorkspace();
		// Returns true if the servo are being stopped for some reason.
		bool isStopped();

        std::map<std::string,Servo*> servos_;
        std::map<int,Servo*> servo_by_pos_;
        std::string ethernetport_;
        std::atomic<int> cycletime_ms_;
        EthercatStates currentstate_;
        char IOmap_[2048];
        std::atomic<bool> PDO_input_enable_{false};
        std::atomic<bool> PDO_output_enable_{false};
        std::atomic<bool> is_connected_{false};
        std::future<void> async_future_;
        bool has_done_SDO_and_PDO_configuration_{false};
        std::atomic<bool> logging_enabled_{false};
		pthread_mutex_t stopallMx;
		bool stopped = true;

    };
}
