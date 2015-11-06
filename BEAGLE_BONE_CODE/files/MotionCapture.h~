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
#include "math.h"
#include "float.h"
#include "Matrix.h"
#include "Kalman.h"
#include "SuperKalman.h"
#include <ctime>

#include "AngleContainer.h"

#include <sys/time.h>
#include <time.h>

using namespace std;

#define PACKET_TYPE_MASK 		0x04
#define FULL_PACKET_SIZE 		17
#define HALF_PACKET_SIZE		11
#define QUARTER_PACKET_SIZE		6
#define FULL_PACKET_SIZE_CHKSUM		FULL_PACKET_SIZE + 1
#define NUM_SENSORS 			0x0B
#define T_SAMPLE			0.016402875 // 60.96Hz --- (0.033227 for 30Hz)
#define NTH_COMPASS_READ		6 	// read every nth compass.  Determined by T_SAMPLE above.

#define ACCEL_FLAG			0x80
#define COMPASS_FLAG			0x40
#define ORIEN_FLAG			0x20

#define GYRO_THRESHOLD			0.007 // Jitter in gyro don't care if < 1deg/sec
#define GYRO_SUPPLY_MILLIVOLTS 		3300.0
#define GYRO_SENSITIVITY_MILLIVOLTS	0.83

#define GRAPH_MULTIPLE			25.0

#define GYRO_VREF			386 // in hex is 0x182

#define ROT_UP				0x0
#define ROT_DOWN			0x1

#define TORSO				0x09 
#define RU_ARM				0x01
#define RL_ARM				0x03
#define RU_LEG				0x05
#define RL_LEG				0x07
#define LU_ARM				0x00
#define LL_ARM				0x04
#define LU_LEG				0x06
#define	LL_LEG				0x08

#define UPPERTHRESH			2

#define buf_len				4000
#define PI				3.1415927



// Set to 36 to include the raw angles in the output
#define MEMBLOCK_LEN		18
class MotionCapture
{
public:
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
	
	// setQuat handles the setting of the quaternions and which sensor component
	// generated this.
	// This is done to allow selection of the quaernion sensor source as a experimental
	// aspect of the design....
	void setQuat(unsigned char myID, Quaternion myT, bool isCompass);
	bool validID(unsigned char myID);
	Quaternion* getQuat(unsigned char myID);
	QVector* getQvectorOld(unsigned char myID);
	void setQvectorOld(QVector &RHS, unsigned char myID);
	bool compareQVect(QVector &LHS, QVector &RHS, float myThresh);

	void printResults(unsigned char myID, const QVector &accel);


void floatToChar(const float &in, unsigned char out[2]);

void charToFloat(unsigned char in[2], float &out);

void Rotate_To_TorsoReference(unsigned char myID);

bool getTorsoFlag();
private:
	static const float GYRO_CONVERSION_CONST = ((GYRO_SUPPLY_MILLIVOLTS / GYRO_SENSITIVITY_MILLIVOLTS) / 1023.0); //* T_SAMPLE;
	//static const float PI = 3.1415927;
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
	//Quaternion ReferenceOrientationQuat[NUM_SENSORS];
	Quaternion OutputOrientationQuat[NUM_SENSORS];
	Quaternion TORSO_Z;
	Quaternion TORSO_Z_Container[5];
	int TorsoWaitCount;
	Quaternion TORSO_Z_RIGHTSIDE;
	Quaternion TORSO_Z_LEFTSIDE;

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
	// a threshold JHD 7_2_13...OK NOT USED 9_24_14
	QVector OldAccelVect[NUM_SENSORS];
	QVector NewAccelVect[NUM_SENSORS];	
	QVector CompassVect[NUM_SENSORS];
	QVector BaselineOffsetPoint[NUM_SENSORS];
	bool SensorSet[NUM_SENSORS];

	// ONLINE CALIBRATION OF GYROS
	#define  GYROAVG_WINDOW 100 	
	QVector ONLINE_GYRO_CAL_VEC[NUM_SENSORS][GYROAVG_WINDOW];
	int ONLINE_GYRO_CAL[NUM_SENSORS];
	QVector ONLINE_GYRO_CAL_COMPLETE[NUM_SENSORS];
	
	// Used to add in the offset vector for the limbs
	// in reference to the Torso
	QVector OffsetVect[NUM_SENSORS];
	bool TORSO_FLAG;
	ofstream myOutputStream;
	int TotalBuffSize;
	
	QVector LU_ARM_AVG[3];
	int LU_ARM_AVG_IND;
	int debug_Counter;
	
	unsigned char* myFileBuff;	
	bool isCompass_Update(unsigned char myID);
	
	// JHD 4/14
	// Used to hold the raw before conversion angles of the gyro / compass / accell
	// format is as follows <GXX><GYY><GZZ><CXX><CYY><CZZ><AXX><AYY><AZZ>
	AngleContainer RawAngleBlock[NUM_SENSORS];
	
	unsigned char memblock[MEMBLOCK_LEN];
	
	
};

#endif
