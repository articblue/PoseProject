#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "QVector.h"
#include <string>
#include <sstream>
#include <math.h>

using namespace std;

class Quaternion
{
public:
	Quaternion();
	Quaternion(float scalar, QVector inputVector);
	Quaternion(float, float, float);
	Quaternion operator*(Quaternion);
	Quaternion operator+(Quaternion);
	Quaternion operator/(float num);

	Quaternion conjugate();
	void normalize();

	void toEuler(float &roll, float &pitch, float &yaw);
	void fromEuler(const float &roll, const float &pitch, const float &yaw);

	Quaternion rotate(Quaternion, QVector);
	QVector toVector() const;
	const char* toString();
	void scaleQuat(float num);
	float getScalar();

	float w;
	QVector vector;

	void setID(int tID) { ID = tID;};

	Quaternion quaternionFromPoints(QVector P1, QVector P2);
	Quaternion& operator=(const Quaternion &rhs);
	Quaternion& operator+=(const Quaternion &rhs);
	Quaternion& operator-=(const Quaternion &rhs);
	void print(int num);
	QVector cross(QVector, QVector);
	float dot(QVector, QVector);
	
	void set_isCompass(bool a);
	bool query_isCompass();


private:
	static const float PI = 3.1415927;
	int ID;
	bool isCompass;
};

	inline Quaternion operator-(Quaternion lhs, const Quaternion &rhs)
	{
		lhs-=rhs;
		return lhs;
	}


#endif
