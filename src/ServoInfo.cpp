#include "../include/asp_servo_api/ServoInfo.hpp"

namespace asp{
	

	inline int CLIP(int servoCmd)
	{
		if (servoCmd > MAX_V) {
			servoCmd = MAX_V;
		}
		if (servoCmd < -MAX_V) {
			servoCmd = -MAX_V;
		}
		return servoCmd;
	}	

	int ServoInfo::toTicks(double cmdSI) const
	{
		return CLIP((int)(cmdSI*conversion));
	}

	double ServoInfo::SIfromTicks(int ticks) const
	{
		return ((double)ticks)/conversion;
	}
	
	bool ServoInfo::inLimitsTicks(int currentPos) const
	{
		return currentPos >= limits[0] && currentPos <= limits[1];
	}
	
	int ServoInfo::getNumber() const
	{
		return number;

	}

	std::string ServoInfo::getName() const
	{
		return name;
	}

	int ServoInfo::getLlimTicks() const
	{
		return limits[0];
	}

	/**
	* Gets the upper limit (expressed as encoder ticks).
	*/
	int ServoInfo::getUlimTicks() const
	{
		return limits[1];
	}

	int ServoInfo::getPositionTicks() const
	{
		return position;
	}


	double ServoInfo::getPositionSI() const
	{
		return ((double)position)/conversion;
	}

	void ServoInfo::setPositionTicks(int pos){
		position = pos;
	}

}
