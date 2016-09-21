#include <iostream>
#include <unistd.h>

class Pid
{
public:
	float kp;
	float ki;
	float kd;
	float lastError = 0;
	float errorSum = 0;

	Pid(float kp, float ki, float kd)
		: kp{kp},
		ki{ki},
		kd{kd}
	{ }

	float calculate(float currentValue, float setPoint, float deltaTime)
	{
		float error = setPoint - currentValue;
		errorSum += error * deltaTime;
		float errorDerivative = (error - lastError) / deltaTime;
		lastError = error;

		return kp * error + ki * errorSum + kd * errorDerivative;
	}
};

int main()
{
	const int frameRate = 60;
	const int lineWidth = 140;
	float v = 0;
	float x = 0;
	float target = lineWidth / 2;

	Pid p{0.01, 0.01, 0.001};

	while (true) {
		int squares = lineWidth * x / lineWidth;

		std::cout << '\r';

		for (int i = 0; i < lineWidth; i++) {
			if (i == (int)target)
				std::cout << '|';
			else if (i < squares)
				std::cout << '#';
			else
				std::cout << ' ';
		}

		// without '\n' std::cout won't flush automatically
		std::cout << std::flush;

		// NOTE(Andrey): Uncomment this to see only the current slider
		std::cout << std::endl;

		v += p.calculate(x, target, 1.0 / frameRate);
		v += -0.1;
		x += v;

		usleep(1000 / frameRate * 1000);
	}

	return 0;
}
