/******************************************************************************
* www.pvl.vn
* > g++ -o MCP4822_DAC_WIRINGPI MCP4822_DAC_WIRINGPI.cpp -lwiringPi
* > sudo ./MCP4822_DAC_WIRINGPI
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
//#include <SerialStream.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>
//------------------------------------------------------------------------------DAC SPI
const int CHANNEL = 0; //PIN08 - CE0
const int LDAC = 5; //PIN24
const int BUTTON = 4; //PIN23
void write_dac_left(int value) {
int dt;
dt = value;
uint8_t buffer[2];
//buffer[0] = dt >> 8 | 0x10;
//buffer[1] = dt & 0xff;

buffer[0] = dt >> 8 | 0x00;
buffer[1] = dt & 0x11;
digitalWrite(LDAC, HIGH); // On
wiringPiSPIDataRW(CHANNEL, buffer, 2);
digitalWrite(LDAC, LOW); // Off
}
void write_dac_right(int value) {
int dt;
dt = value;
uint8_t buffer[2];
buffer[0] = dt >> 8 | 0x90;
buffer[1] = dt & 0xff;
digitalWrite(LDAC, HIGH); // On
wiringPiSPIDataRW(CHANNEL, buffer, 2);
digitalWrite(LDAC, LOW); // Off
}
int main() {
//--------------------------------------------------------------------------WIRINGPI SETUP
wiringPiSetup();
//--------------------------------------------------------------------------DAC SETUP SPI
pinMode(LDAC, OUTPUT);
pinMode(CHANNEL, OUTPUT);
wiringPiSPISetupMode(CHANNEL, 15000000, 0);
wiringPiSPISetup(CHANNEL, 20000000); //20Mhz
printf("STARTING\n");
//--------------------------------------------------------------------------WHILE TRUE
while (1) {
write_dac_left(1024);
write_dac_right(3300);
}
return 0;
}
