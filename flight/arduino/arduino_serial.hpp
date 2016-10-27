//
// Based on arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
//

// Modifed by Andrey Leshenko, 2016

int arduino_serial_open(const char* serialport, int baud);
int arduino_serial_close(int fd);
