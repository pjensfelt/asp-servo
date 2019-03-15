#pragma once
#include "csv.hpp"
#include "timespec.hpp"
#include "ServoInfo.hpp"
#include <cmath>

// Defined in csv.hpp
extern asp::ServoCollection servoCollection;
extern int writeCnt;
extern sem_t semaphore;	 			// Used to wake up the watchdog thread
extern pthread_mutex_t stopallMx; 	// Used to avoid more than one thread trying to stop the servos at the same time
extern bool stopped;				// At the beginning, the servos are stopped
extern const int CTRL_LOOP_MS;
extern const int MS_TO_NS;
const int WATCHDOG_LOOP_MS = 2*CTRL_LOOP_MS; // [ms] 


/**
* Implements the watchdog loop: periodically checks if a variable has been updated,
* which means that 
*/
void *watchdogFct(void *args){

	//registerHandlers();
	std::cout << "Watchdog function up" << std::endl;
	
	// Variables
	asp::ServoCollection *servo_collection = (asp::ServoCollection *)args;
	int prevCount = -2;
	int readVal = -1;
	
	struct timespec oldTime, newTime, diff;
	struct timespec tspec, remaining;
    int64_t cycletime_ns = WATCHDOG_LOOP_MS * MS_TO_NS;	// From ms to ns
    long            ms; // Milliseconds
    time_t          s;  // Seconds
	
	// Wait the control thread to be operative
	sem_wait(&semaphore);
	std::cout << "Now watchdog awake " << std::endl;
    
    // Initiali time
    //clock_gettime(CLOCK_REALTIME, &newTime);
    clock_gettime(CLOCK_MONOTONIC, &tspec);	
	do{		
		// Adding the cycletime to the timespec
        timespec_add(&tspec, cycletime_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tspec, &remaining);
        	   
		prevCount = readVal;
		readVal = writeCnt;
     
	}while(readVal != prevCount);
	
	// If we're here, the ctrl_fucntion hasn't updated the write count in a while -> emergency
	std::cout << "Stopping. Watchdog TIMEOUT after " << WATCHDOG_LOOP_MS << " ms,  stopping all" << std::endl;
	servoCollection.stopAll();
	
	std::cout << "Watchdog exiting "<< std::endl;
	pthread_exit(0);	
}
