/*
data: 19/09/2016
Autor: Marco Tulio Chella
referencia: https://github.com/omaflak/GY-521-Raspberry-Pi-C-
para compilar:
g++ dac_gyro.cpp -lwiringPi libSensor.cpp KalmanFilter.cpp
*/

#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <stdio.h>
#include <cmath>
#include "libSensor.h"
#include "KalmanFilter.h"
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

// 加速度/ジャイロセンサーの制御変数。
KalmanFilter gKfx, gKfy; // カルマンフィルタ。
float gCalibrateY; // 初期化時の角度。（＝静止角とみなす）
long gPrevMicros; // loop()間隔の計測用。

Sensor gyro;

// 倒立振子の制御変数。
float gPowerP, gPowerI, gPowerD; // 現在出力値とPID成分。

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


void setup() {
  // 重力加速度から求めた角度をカルマンフィルタの初期値とする。

  float ax,ay,az;

  ax = gyro.getAngleX();
  ay = gyro.getAngleY();
  az = gyro.getAngleZ();


  float degRoll  = atan2(ay, az) * 57.29578;
  float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * 57.29578; //rad to deg
  gKfx.setAngle(degRoll);
  gKfy.setAngle(degPitch);
  gCalibrateY = degPitch;
  gPrevMicros = micros();
}


int main()
{
    if (wiringPiSetup() == -1)
        return -1;

    if(wiringPiSetupGpio() == -1)                //GPIO初期化
    return -1;

    //DAC SETUP SPI
    pinMode(LDAC, OUTPUT); //GPIOピンのモード設定pinMode(ピン番号,モード(Output/Input))
    pinMode(CHANNEL, OUTPUT);

    wiringPiSPISetup(CHANNEL, 2000000); //20Mhz
    printf("STARTING\n");
    float ax,ay,az,gx,gy,gz;

    while(1){
         // 重力加速度から角度を求める。
    ax = gyro.getAngleX();
    ay = gyro.getAngleY();
    az = gyro.getAngleZ();
    float degRoll  = atan2(ay, az) * 57.29578;
    float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * 57.29578;

  // ジャイロで角速度を求める。
    gx = gyro.getGyroX();
    gy = gyro.getGyroY();
    gz = gyro.getGyroX();

    float dpsX = gx / 131.0; // LSB sensitivity: 131 LSB/dps @ ±250dps
    float dpsY = gy / 131.0;
    float dpsZ = gz / 131.0;

    // カルマンフィルタで角度(x,y)を計算する。
    long curMicros = micros();
    float dt = (float)(curMicros - gPrevMicros) / 1000000; // μsec -> sec
    gPrevMicros = curMicros;
    float degX = gKfx.calcAngle(degRoll, dpsX, dt);
    float degY = gKfy.calcAngle(degPitch, dpsY, dt);
    degY -= gCalibrateY;

    printf("degPitch=%f   degRoll=%f     \n",degRoll,degPitch);
	
    if(degPitch <= 50  &&  degPitch  >  40  ||  degPitch  <  -40  &&  degPitch  >=-50){
        dac_value(700);
    }else if(degPitch <= 40  ||  degPitch  <  -40){
        dac_value(1000);
    }else{
        dac_value(0);
    }
	printf("value=%d\n",value);  
	   
    }
}
