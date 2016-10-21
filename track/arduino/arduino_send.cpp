//sending PID results to arduino

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "arduino_serial.hpp"
#include "arduino_send.hpp"

const char *possible_serial_devices[] = {"/dev/ttyACM0", "/dev/ttyACM1"};

int send_to_serial(int serial, unsigned char roll, unsigned char pitch, unsigned char throttle, unsigned char yaw)
{
	unsigned char cmd[5] = {0xff, roll, pitch, throttle, yaw};
	if (write(serial, cmd, 5) != 5)
	{
		return -1;
	}
	return 0;
}

static int clamp_int(int value, int min, int max)
{
	if (value > max) return max;
	if (value < min) return min;
	return value;
}

void control_drone(int roll, int pitch, int throttle, int yaw)
{
	int serialfd = -1;

	//
	// open arduino
	//

	{
		int serial_count = sizeof(possible_serial_devices) / sizeof(*possible_serial_devices);

		for (int i = 0; i < serial_count; i++)
		{
			serialfd = arduino_serial_open(possible_serial_devices[i], 115200);
			if (serialfd >= 0)
				break;
		}

		if (serialfd < 0)
		{
			printf("ERROR: couldn't connect to any of the serial devices specified.\n"
					"Commands WILL NOT be sent.\n"
					"\n"
					"Make sure your device is connected.\n"
					"\n"
					"Devices checked:\n");
			for (int i = 0; i < serial_count; i++)
			{
				printf("    %s\n", possible_serial_devices[i]);
			}
			printf("\n"
					"To add devices to this list, update the `possible_serial_devices` variable and recompile.\n"
					"\n"
					"press ENTER to continue");
			getchar();
		}
	}

	//
	// send data to arduino
	//

	{
		if (serialfd >= 0)
		{
			int ret = send_to_serial(serialfd, roll, pitch, throttle, yaw);
			if (ret < 0)
			{
				fprintf(stderr, "ERROR: Couldn't send commands to serial.\n");
			}
		}
	}

	//
	//close arduino
	//

	if (serialfd >= 0)
	{
		arduino_serial_close(serialfd);
	}
}
