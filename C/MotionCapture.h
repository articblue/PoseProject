#ifndef MOTIONCAPTURE_H
#define MOTIONCAPTURE_H

#include <string.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>      /*Unix standard function definitions*/
#include <fcntl.h>      /*File control definitions*/
#include <cerrno>      /*Error number definitions*/
#include <termios.h>      /*POSIX terminal control definitions*/
#include "QVector.h"
#include "Quaternion.h"
#include <cmath>
#include "float.h"
#include "Matrix.h"
#include "Kalman.h"
#include "SuperKalman.h"
#include <ctime>

#include <sys/time.h>
#include <time.h>

using namespace std;

#define PACKET_TYPE_MASK 		0x04
#define FULL_PACKET_SIZE 		17
#define HALF_PACKET_SIZE		11
#define QUARTER_PACKET_SIZE		6
#define FULL_PACKET_SIZE_CHKSUM		FULL_PACKET_SIZE + 1
#define NUM_SENSORS 			11
#define T_SAMPLE			0.016402875 // 60.96Hz --- (0.033227 for 30Hz)
#define NTH_COMPASS_READ		6 	// read every nth compass.  Determined by T_SAMPLE above.

#define ACCEL_FLAG			0x80
#define COMPASS_FLAG			0x40
#define ORIEN_FLAG			0x20

#define GYRO_SUPPLY_MILLIVOLTS 		3300.0
#define GYRO_SENSITIVITY_MILLIVOLTS	0.83

#define GRAPH_MULTIPLE			25.0

#define GYRO_VREF			386 // in hex is 0x182

#define ROT_UP				0x0
#define ROT_DOWN			0x1

//#define TORSO				0 // Torso Sensor
#define TORSO				9 
#define RU_ARM				1
#define RL_ARM				3
#define RU_LEG				5
#define RL_LEG				7
#define LU_ARM				0
#define LL_ARM				4
#define LU_LEG				6
#define	LL_LEG				8

#define UPPERTHRESH			2

#define buf_len				4000

struct QuaternionArray {
	Quaternion quats[9];
};

class MotionCapture
{
public:
	QuaternionArray fillAndGetQuatArray();
	MotionCapture();
	int openFile(char *name);
	void closeFile();
	int openSerialPort(char* path, bool read);
	bool readConfigurationFile();

	void getPacket(int &len, char packet[FULL_PACKET_SIZE_CHKSUM], unsigned char &packet_flags, const bool &file_encrypted);
	char getID(char* packet);
	bool check_still(char* packet, QVector &data);
	QVector extractAccel(unsigned char myID, char* packet, float &pitch, float &roll);
	QVector extractGyro(unsigned char myID, char* packet);
	QVector extractCompass(unsigned char myID, char* packet, int packet_size);
	bool printGyroOffsets(const unsigned char &myID, const unsigned char &x, const unsigned char &y, const unsigned char &z, const unsigned char &msb);
	QVector reconcile(QVector orientationVector, QVector stillVector);
	QVector reconcile(QVector orientationVector, QVector stillVector, QVector compassVector);
	float hex_to_dec(char msb, char lsb);
	
	void applyEKF(const unsigned char &myID, Quaternion& compass, QVector &gyro, const QVector &accel, bool has_compass, const int &g);
	void applyEKF(const unsigned char &myID, QVector &gyro, const QVector &accel, bool has_compass, const int &g);

void filterandRotate(const unsigned char &myID, const float &heading, const float &pitch, const float &roll, const bool &is_level, const bool &filter);
void extractAngles(float &level_H, float &level_P, float &level_R, float &output_H, float &output_P, float &output_R );
void rotEuler90(float& yaw, float& pitch, float& roll, const int &dir);

	void setQuat(unsigned char myID, Quaternion myT);
	bool validID(unsigned char myID);
	Quaternion* getQuat(unsigned char myID);
	QVector* getQvectorOld(unsigned char myID);
	void setQvectorOld(QVector &RHS, unsigned char myID);
	bool compareQVect(QVector &LHS, QVector &RHS, float myThresh);

	void printResults(const unsigned char &myID, const QVector &accel);


void floatToChar(const float &in, unsigned char out[2]);

void charToFloat(unsigned char in[2], float &out);

void Rotate_To_TorsoReference(unsigned char myID);
private:
	static constexpr float GYRO_CONVERSION_CONST = ((GYRO_SUPPLY_MILLIVOLTS / GYRO_SENSITIVITY_MILLIVOLTS) / 1023.0f); //* T_SAMPLE;
	const float PI = 3.1415927f;
	unsigned char serial_buf[buf_len];
	unsigned char tmp_buf[buf_len];
	int readFile(char *serialArray, unsigned char &packet_flags, const bool &file_encrypted);
	int readSerial(char *serialArray, unsigned char &packet_flags);

	FILE *inputFile;
	bool read_from_serial;

	Kalman *kalman_level;
	Kalman *kalman_upright;

	SuperKalman extended_kalman[NUM_SENSORS];

	int serialDescriptor;
	int serialLength;
	int serialOutput;

	float loc_heading, loc_pitch, loc_roll;
	float correct_heading, correct_pitch, correct_roll;

	int gyroX[NUM_SENSORS], gyroY[NUM_SENSORS], gyroZ[NUM_SENSORS];
	int counter[NUM_SENSORS];

	char gyro_calibration_x[NUM_SENSORS];
	char gyro_calibration_y[NUM_SENSORS];
	char gyro_calibration_z[NUM_SENSORS];
	Quaternion orientationQuat[NUM_SENSORS];
	Quaternion orientationQuatLAG[NUM_SENSORS];
	Quaternion ReferenceOrientationQuat[NUM_SENSORS];
	Quaternion OutputOrientationQuat[NUM_SENSORS];
	Quaternion BaselineOffsetQuaternion[NUM_SENSORS];

	Quaternion X_Axis[NUM_SENSORS];
	Quaternion Y_Axis[NUM_SENSORS];
	Quaternion Z_Axis[NUM_SENSORS];
	Quaternion X_Axis_Baseline[NUM_SENSORS];
	Quaternion Y_Axis_Baseline[NUM_SENSORS];
	Quaternion Z_Axis_Baseline[NUM_SENSORS];

	Quaternion TORSO_ABS;
	Quaternion LU_LEG_ABS;
	Quaternion RU_LEG_ABS;
	Quaternion LU_ARM_ABS;
	Quaternion RU_ARM_ABS;

	// Yeah doensn't really need to be all the sensors..just easier
	Quaternion BaselineOffsetQuaternionLS[NUM_SENSORS];
	// Yeah doensn't really need to be all the sensors..just easier
	Quaternion BaselineOffsetQuaternionRS[NUM_SENSORS];
	int WaitCounter;
	// These are the accel vectors for a previous and current
	// acceleration
	// These will be used to mitigate noise contributions at the
	// 2 LSB level
	// Also these will be used to prevent spikes in accel above 
	// a threshold JHD 7_2_13
	QVector OldAccelVect[NUM_SENSORS];
	QVector NewAccelVect[NUM_SENSORS];	
	QVector CompassVect[NUM_SENSORS];
	QVector BaselineOffsetPoint[NUM_SENSORS];
	bool BaselineOffsetSet[NUM_SENSORS];
	// Used to add in the offset vector for the limbs
	// in reference to the Torso
	QVector OffsetVect[NUM_SENSORS];
	bool TORSO_FLAG;
	ofstream myOutputStream;
	int TotalBuffSize;
	
	QVector LU_ARM_AVG[3];
	int LU_ARM_AVG_IND;
	int debug_Counter;
};

#endif
