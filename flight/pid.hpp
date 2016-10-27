#pragma once

class Pid
{
public:
	float kp;
	float ki;
	float kd;
	float lastError = 0;
	float errorSum = 0;
	float min = -1;
	float max = 1;

	Pid() {};

	Pid(float kp, float ki, float kd)
		: kp{kp},
		ki{ki},
		kd{kd}
	{ };

	float calculate(float currentValue, float setPoint, float deltaTime);

	float calculate(float error, float deltaTime);

	void setLimits(float newMin, float newMax);

private:
	float clamp(float val);
};
