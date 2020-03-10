// package: reschy
// module: robot_test_controller
// revision history
// date       | author      | description
// 2015.10.4  | T.W.Kang    | PID control
//
//
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

float _integralError = 0.0;
float _previousError = 0.0;
float _Kp = 4.0;		// proportional output gain value
float _Ki = 1.0;		// integral output gain value
float _Kd = 1.0;		// derivative output gain value

void initPID(float Kp = 4.0, float Ki = 1.0, float Kd = 1.0)
{
	_integralError = 0.0;
	_previousError = 0.0;
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;	
}

float calcPID(float targetValue, float currentValue, float samplingTime)
{
	if(samplingTime == 0.0)
		return 0.0;

	float error = targetValue - currentValue;	// error. ex) 180 degree - current angle
	if(_previousError == 0.0)
		_previousError = error;
	float KpTerm = _Kp * error;					// proportional output
	
	_integralError = _integralError + _integralError * samplingTime;	// error integral
	float KiTerm = _Ki * _integralError;		// integral output

	float derivError = (error - _previousError) / samplingTime;	// delta error
	float KdTerm = _Kd * derivError;			// derivative output

	_previousError = error;
	float control = KpTerm + KiTerm + KdTerm;
	return control;	
} 

float convertValue(float controlPID, float controlRange, float min1, float max2)
{
	float diff = max2 - min1;
	float value = (controlPID / controlRange) * diff + min1;
	return value;
}

