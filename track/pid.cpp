#include "pid.hpp"

float Pid::calculate(float currentValue, float setPoint, float deltaTime)
{
	float error = setPoint - currentValue;
	return calculate(error, deltaTime);
}

float Pid::calculate(float error, float deltaTime)
{
	errorSum += error * deltaTime;
	float errorDerivative = (error - lastError) / deltaTime;
	lastError = error;

	return clamp(kp * error + clamp(ki * errorSum) + kd * errorDerivative);
}

void Pid::setLimits(float newMin, float newMax)
{
	min = newMin;
	max = newMax;
}

float Pid::clamp(float val)
{
	if(val < min) return min;
	if(val > max) return max;
	return val;
}
