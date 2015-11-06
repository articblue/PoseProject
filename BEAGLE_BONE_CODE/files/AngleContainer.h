#ifndef ANGLECONTAINER_H
#define ANGLECONTAINER_H


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
#include <ctime>

#include <sys/time.h>
#include <time.h>

using namespace std;


#define ANGLE_COMPASS 1
#define ANGLE_GYRO 	2
#define ANGLE_ACCEL 	3

class AngleContainer
{
	public:
		AngleContainer();
		AngleContainer(unsigned char id);
		AngleContainer * getAngles();
		void setAngles(float x, float y, float z, int ANGLETYPE);
	
	private:
		unsigned char myID;
		float xC;
		float yC;
		float zC;
	
		float xG;
		float yG;
		float zG;
		
		float xA;
		float yA;
		float zA;
};

#endif
