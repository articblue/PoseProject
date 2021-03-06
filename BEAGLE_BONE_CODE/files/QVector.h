#ifndef QVECTOR_H
#define QVECTOR_H

#include "math.h"
#include <stdio.h>
#include <sstream>
#include <iomanip>

using namespace std;

class QVector
{
public:
	QVector();
	QVector(float, float, float);
 	QVector(const QVector &vec);

	QVector operator+(QVector);
	QVector operator-(QVector);
	QVector operator*(float);
	QVector operator/(float);
	QVector operator+=(QVector);
	QVector vectorCross(QVector V2);
	float vectorDot(QVector V2);
	void setXYZ(float, float, float);
	float getX();
	float getY();
	float getZ();
	QVector unitVector();
	float magnitude();
	QVector emptyVector();
	void setEmpty(bool);
	bool isEmpty();
	const char* toString();

	float x;
	float y;
	float z;
	bool empty;
	bool firstUpdate;
};

#endif
