#pragma once
#include <string>


#define MAX_V 		1310720

/*
* Defines useful information to handle the commands to the servomotors.
*/
namespace asp {

	class ServoInfo
	{
		private:
		struct Limits
		{
			int limits[2];
			int& operator[](int i) {return limits[i];}
			int const& operator[](int i) const {return limits[i];}	
			Limits(){
				limits[0] = 0;
				limits[1] = 0;
			}

			Limits(int llim, int ulim){
				limits[0] = llim;
				limits[1] = ulim;
			}
		};
			
		std::string name;			// Servo name, either s1/s2,
		double 		conversion; 	// To pass from ticks to m or rads
		std::string unit; 			// Either "m" or "rad"
		Limits 		limits; 		// Lower and upper limits expressed in ticks.
		int 		number;			// Position related to the master (s1->1, s2->2...)
		uint64_t	position = 0; 		// Position in encoder ticks. Updated each time the callback is executed
		uint8_t		outOfLimits = 0;// Will be -1 if out of lower limits, 0 if in liimts, 1 otherwise
		public:
		
		/**
		* Creates a new servoInfo object.
		*/
		ServoInfo(){};
		ServoInfo(std::string _name, int _number, std::string _altname, double _conversion, std::string _unit, int lLim, int uLim)
		{
			name = _name;
			number = _number;
			conversion = _conversion;
			unit = _unit;
			limits = Limits(lLim, uLim);	
		}

		/**
		* Converts a command in the S.I. system to a command in ticks. 
		*/
		int toTicks(double cmdSI) const;
		
		/**
		* Converts the encoder value to a S.I. measure (in meters/rad).
		*/
		double SIfromTicks(int ticks) const;

		/**
		* Checks if the current position is inside the defined limits. 
		* The position is expressed as encoder ticks.
		*/
		bool inLimitsTicks(int currentPos) const;

		/**
		* Returns the servo position relative to the master (i.e. 1 for s1, 2 for s2...).
		*/
		int getNumber() const;
		
		/**
		* Returns the servo name (i.e. 1 for s1, 2 for s2...).
		*/
		std::string getName() const;

		/**
		* Gets the lower limit (expressed as encoder ticks).
		*/
		int getLlimTicks() const;

		/**
		* Gets the upper limit (expressed as encoder ticks).
		*/
		int getUlimTicks() const;

		/**
		* Returns the lastly saved position of the servo motor as encoder ticks.
		*/
		int getPositionTicks() const;

		/**
		* Returns the lastly saved position of the servo motor in the S.I. system.
		*/
		double getPositionSI() const;

		/**
		* Saves the lastly read position of the servo motor.
		*/
		void setPositionTicks(int pos);
	};
}

