/*
 * IMU.h
 *
 *  Created on: Jul 5, 2012
 *      Author: Dan Chianucci
 */

#ifndef IMU_H_
#define IMU_H_



#include "ADXL345/ADXL345.h"
#include "HMC5883L/HMC5883L.h"
#include "ITG3200/ITG3200.h"


#define sampleFreq	25.0f		/// sample frequency in Hz
#define betaDef		0.041f		/// 2 * proportional gain


/**
 * The Following defines are used to map Sensor axis' to a common board axis
 * For Example:
 * 		GX means the X-Axis of the Gyroscope
 * 		GY means the Y-Axis of the Gyroscope
 * 		GZ means the Z-Axis of the Gyroscope
 *
 * 		0 means the X-axis of the Board
 * 		1 means the Y-axis of the Board
 * 		2 means the Z-axis of the Board
 *
 * 		Therefore
 * 		GX 0 would mean that the gyroscope's X-axis is the same as the boards X Axis.
 * 		GX 1 would mean that the gyroscope's X-axis corresponds to the boards Y Axis.
 */

#define GX 0
#define GY 1
#define GZ 2

#define GXS 1
#define GYS 1
#define GZS 1

#define MX 1
#define MY 0
#define MZ 2

#define MXS 1
#define MYS 1
#define MZS 1

#define AX 1
#define AY 0
#define AZ 2

#define AXS -1
#define AYS 1
#define AZS 1



class IMU {
public:
	/**
	 * Constructs and Initializes the IMU
	 */
	IMU();

	void Update();
	void init();
	void init(uint8_t accAddr, uint8_t gyrAddr, uint8_t magAddr);

	void getValuesRaw(int16_t * accV, int16_t * gyrV, int16_t * magV);
	void getValuesScaled(float * accV, float * gyrV, float * magV);

	void getQuaternion(float * quatArr);
	void getData(float *q, float *a, float *g, float *m );

private:
	ADXL345  acc; 	/// The Accelerometer
	HMC5883L mag; 	/// The Magnetometer
	ITG3200  gyr; 	/// The Gyroscope

	//float exInt, eyInt, ezInt;  // scaled integral error


	float gyrOffsets[3];	///Raw Offsets of gyroscope
	int16_t magMaxs[3];		///Raw Maxes of Magnetometer
	float magScales[3];	///Raw Scale Factors of Magnetometer
	float accOffsets[3];	///Raw Acc Ofsets

	float beta;				/// algorithm gain
	float q0,q1,q2,q3;		/// quaternion of sensor frame relative to auxiliary frame

	int16_t accRaw[3];
	int16_t gyrRaw[3];
	int16_t magRaw[3];			///Raw Sensor Values

	float accScaled[3];
	float gyrScaled[3];
	float magScaled[3];	///Scaled Sensor Values

	/**
	 * Updates the MARG
	 * @param  gx , gy , gz   Gyroscope Axis in radians per second
	 * @param  ax , ay , az   Accelerometer Axis, units are irrelevant
	 * @param  mx , my , mz   Magnetometer Axis , units are irrelevant
	 */
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	/**
	 * Updates the IMU, Used by MadgwickAHRSUpdate()
	 * @param  gx , gy , gz   Gyroscope Axis in radians per second
	 * @param  ax , ay , az   Accelerometer Axis, units are irrelevant
	 */
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

	void calibrateAcc();
	void calibrateMag(unsigned char gain);
	void calibrateGyr();

	float invSqrt(float x);






};

#endif /* IMU_H_ */
