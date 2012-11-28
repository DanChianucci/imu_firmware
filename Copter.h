#ifndef Copter_H_
#define Copter_H_
#include "Arduino.h"


#include "IMU.h"

IMU   imu;				/// The IMU calculator
float gV[3];			/// Gyroscope Measurments in rad/sec
float aV[3];			/// Accelerometer Measurments in milli g
float mV[3];			/// Mag Measurments in ???
float qV[4];			/// The Quaternion Vector


#ifdef __cplusplus
	extern "C" {
#endif

void loop();
void setup();

#ifdef __cplusplus
	} // extern "C"
#endif

void printData();
void printQuat();
void getCommands();


#endif /* Copter_H_ */
