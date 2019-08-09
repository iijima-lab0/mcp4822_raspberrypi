//g++ SPI.cpp -o test -lwiringPi 

#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <iostream>
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

using namespace std;
const int CHANNEL = 0; 
const int LDAC = 26; //GPIO26:PIN37

void dac_value(int value) {
int dt;

dt = value;
uint8_t buffer[2];

buffer[0] = dt >> 8 | 0x30;
buffer[1] = dt & 0xFF;

digitalWrite(LDAC, 0); //SS信号をLOW出力にして通信開始
wiringPiSPIDataRW(CHANNEL, buffer, 2);
digitalWrite(LDAC, 1); //SS信号をHIGH出力にして通信終了

}

int main(){
    if (wiringPiSetup() == -1) //WiringPiの初期化　問題があった場合は-1を返す
        return -1; 

    if(wiringPiSetupGpio() == -1)                //GPIO初期化
        return -1;

    //DAC SETUP SPI
    pinMode(LDAC, OUTPUT); //GPIOピンのモード設定pinMode(ピン番号,モード(Output/Input))
    pinMode(CHANNEL, OUTPUT);

    wiringPiSPISetup(CHANNEL, 2000000); //20Mhz
    printf("STARTING\n");

   while(1){
        dac_value(2000);
        }

    return 0;
}
