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

/**
 * IMU class takes care of updating the orientation of the board, it also
 * handles Sensor Calibration / initialization, and aligning all sensor axis to
 * a common orthagonal coorinate system.
 */
class IMU {
public:
	/**
	 * Constructs and Initializes the IMU
	 */
	IMU();

	/**
	 * Updates the IMU orientation. Must be called regularly in order to work
	 * properly
	 */
	void Update();

	/**
	 * Initializes the IMU with the Default I2C addresses
	 */
	void init();

	/**
	 * Initializes the IMU with the given I2C addresses
	 * @param accAddr the Accelerometer's address
	 * @param gyrAddr the Gyroscope's address
	 * @param magAddr the Magnetometer's address
	 */
	void init(uint8_t accAddr, uint8_t gyrAddr, uint8_t magAddr);

	/**
	 * Gets the raw values of the sensors.
	 * Note: No Scaling, or alignment is done to the returned values
	 *
	 * All parameters will be modified to contain the sensors values in
	 * terms of the sensors own Coordinate System. [X,Y,Z]
	 *
	 * @param accV an array with room for at least 3 ints16_ts
	 * @param gyrV an array with room for at least 3 ints16_ts
	 * @param magV an array with room for at least 3 ints16_ts
	 */
	void getValuesRaw(int16_t * accV, int16_t * gyrV, int16_t * magV);

	/**
	 * Gets the Scaled Values of the sensors
	 * Each paramerter is modified to hold the sensor values, scaled and alligned too the board's
	 * Coordinate system.
	 *
	 * @param accV an array with room for at least 3 floats
	 * @param gyrV an array with room for at least 3 floats
	 * @param magV an array with room for at least 3 floats
	 */
	void getValuesScaled(float * accV, float * gyrV, float * magV);

	/**
	 * Gets the Quaternion Orientation of the IMU
	 * @param quatArr an Array with room for at least 4 floats
	 */
	void getQuaternion(float * quatArr);

	/**
	 * Gets the Quaternion and all Scaled Sensor Values
	 * @param q	quaternion array of 4 floats
	 * @param a accelerometer array of 3 floats
	 * @param g gyroscope array of 3 floats
	 * @param m magnetometer array of 3 floats
	 */
	void getData(float *q, float *a, float *g, float *m );

	/**
	 * Resets the IMU Quaternion and error Integral
	 */
	void reset();

	//://///////////////////////////////////////////////////////////////////////
	//:                                Default Gains
	//://///////////////////////////////////////////////////////////////////////
	float sampleFreq;	/// sample frequency in Hz
	float halfT;   		/// half the sample period

	float beta;     /// 2 * proportional gain
	float Kp; 		/// proportional gain governs rate of convergence to acc/mag
	float Ki;  		/// integral gain governs rate of convergence of gyro biases

private:
	//://///////////////////////////////////////////////////////////////////////
	//:                       Sensors and Calibration Data
	//://///////////////////////////////////////////////////////////////////////
	ADXL345  acc; 			/// The Accelerometer
	HMC5883L mag; 			/// The Magnetometer
	ITG3200  gyr; 			/// The Gyroscope

	float gyrOffsets[3];	///Raw Offsets of gyroscope
	int16_t magMaxs[3];		///Raw Maxes of Magnetometer
	float magScales[3];		///Raw Scale Factors of Magnetometer
	float accOffsets[3];	///Raw Acc Ofsets

	float exInt, eyInt, ezInt;





	//://///////////////////////////////////////////////////////////////////////
	//:                                 Sensor Values
	//://///////////////////////////////////////////////////////////////////////
	float q0,q1,q2,q3;		/// quaternion of sensor frame relative to init

	int16_t accRaw[3];		///Raw Accelerometer Values
	int16_t gyrRaw[3];		///Raw Gyroscope Values
	int16_t magRaw[3];		///Raw Magnetometer Values

	float accScaled[3];		///Scaled Accelerometer Values
	float gyrScaled[3];		///Sclaed Gyroscope Values
	float magScaled[3];		///Scaled Magnetometer Values

	//://///////////////////////////////////////////////////////////////////////
	//:                                Calibration Methods
	//://///////////////////////////////////////////////////////////////////////
	void calibrateAcc();
	void calibrateMag(unsigned char gain);
	void calibrateGyr();



	//://///////////////////////////////////////////////////////////////////////
	//:                                AHRS Methods
	//://///////////////////////////////////////////////////////////////////////

	/**
	 * Updates the MARG
	 * @param  gx , gy , gz   Gyroscope Axis in radians per second
	 * @param  ax , ay , az   Accelerometer Axis, units are irrelevant
	 * @param  mx , my , mz   Magnetometer Axis , units are irrelevant
	 */
	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
			float az, float mx, float my, float mz);

	/**
	 * Updates the IMU, Used by MadgwickAHRSUpdate()
	 * @param  gx , gy , gz   Gyroscope Axis in radians per second
	 * @param  ax , ay , az   Accelerometer Axis, units are irrelevant
	 */
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
			float az);

	/**
	 * Calulates an approximation of the inverse squar root of x
	 * @param x the number to calulate the inverse squar root of
	 * @return the inverse squar root of x
	 */
	float invSqrt(float x);


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
	 *
	 *
	 *  The Defines with S at the end stand for the sign of the axis after the alignment to the board axis.
	 *  For Example:
	 *  	GXS 1 means the the board X-axis needs to be negated for the gyro.
	 */

	enum AXIS_VALUE
	{
		X=0,
		Y=1,
		Z=2
	};


	/**
	 * Maps the Sensor Axis to the Board Axis
	 */
	static const int GXA = X;	  //Gyr
	static const int GYA = Y;
	static const int GZA = Z;

	static const int MXA = Y;     //Mag
	static const int MYA = X;
	static const int MZA = Z;

	static const int AXA = Y;     //Acc
	static const int AYA = X;
	static const int AZA = Z;

	/**
	 * Determines whether a Board Axis needs to be negated
	 */
	static const int GXS =  1;
	static const int GYS =  1;
	static const int GZS =  1;

	static const int MXS =  1;
	static const int MYS = -1;
	static const int MZS =  1;

	static const int AXS = -1;
	static const int AYS =  1;
	static const int AZS =  1;
};

#endif /* IMU_H_ */
