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
#define VREF 2.048 // MCP4822電圧リファレンス[V]

using namespace std;

//------------------------------------------------------------------------------DAC SPI
const int CHANNEL = 1; 
const int LDAC = 26; //PIN37
//const int BUTTON = 4; //PIN23

void write_dac_left(int value) {
int dt;
dt = value;
uint8_t buffer[2];

//----------------------------------------------------------------------------原文
//buffer[0] = dt >> 8 | 0x10;
//buffer[1] = dt & 0xff;

//yosagenaatai
//buffer[0] = dt >> 8 | 0xFF;
//buffer[1] = dt & 0x30;

//----------------------------------------------------------------------------
buffer[0] = dt >> 8 | 0xFF;
buffer[1] = dt & 0x30;

digitalWrite(LDAC, 0); //SS信号をLOW出力にして通信開始
wiringPiSPIDataRW(CHANNEL, buffer, 2); //データ送受信
digitalWrite(LDAC, 1); //SS信号をHIGH出力にして通信開始

//----------------------------------------------------------------------------原文
//digitalWrite(LDAC, HIGH); // On
//wiringPiSPIDataRW(CHANNEL, buffer, 2); //データ送受信
//digitalWrite(LDAC, LOW); // Off
//----------------------------------------------------------------------------
}

void write_dac_right(int value) {
int dt;
dt = value;
uint8_t buffer[2];
//----------------------------------------------------------------------------原文
//buffer[0] = dt >> 8 | 0x90;
//buffer[1] = dt & 0xff;
//----------------------------------------------------------------------------
buffer[0] = dt >> 8 | 0x30;
buffer[1] = dt & 0xFF;

digitalWrite(LDAC, 0); //SS信号をLOW出力にして通信開始
wiringPiSPIDataRW(CHANNEL, buffer, 2);
digitalWrite(LDAC, 1); //SS信号をHIGH出力にして通信開始

//----------------------------------------------------------------------------原文
//digitalWrite(LDAC, HIGH); // On
//wiringPiSPIDataRW(CHANNEL, buffer, 2);
//digitalWrite(LDAC, LOW); // Off
//----------------------------------------------------------------------------
}

int main(){
    if (wiringPiSetup() == -1) //WiringPiの初期化　問題があった場合は-1を返す
        return -1; 

    if(wiringPiSetupGpio() == -1)                //GPIO初期化
        return -1;

    //DAC SETUP SPI
    pinMode(LDAC, OUTPUT); //GPIOピンのモード設定pinMode(ピン番号,モード(Output/Input))
    pinMode(CHANNEL, OUTPUT);
    //wiringPiSPISetupMode(CHANNEL, 15000000, 0);
    wiringPiSPISetup(CHANNEL, 2000000); //20Mhz
    printf("STARTING\n");
    //----------------------------------------------------------------------------原文
    //while(1){
      //  write_dac_left(1024);
        //write_dac_right(3300);
       // }
        //----------------------------------------------------------------------------
   while(1){
        write_dac_left(512);
        write_dac_right(300);
        }

    return 0;
}
//----------------------------------------------------------------------------





