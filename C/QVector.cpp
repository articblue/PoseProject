#include "QVector.h"

QVector::QVector()
{
	x = 0;
	y = 0;
	z = 0;
	empty = 0;
	firstUpdate = 1;	
}

QVector::QVector(float _x, float _y, float _z)
{
	x = _x;
	y = _y;
	z = _z;
	empty = 0;
	firstUpdate = 1;
}

QVector::QVector(const QVector &vec)
{
	x = vec.x;
	y = vec.y;
	z = vec.z;
}

QVector QVector::operator+(QVector v1)
{
	QVector v2;
	v2.x = this->x + v1.x;
	v2.y = this->y + v1.y;
	v2.z = this->z + v1.z;
	return v2;
}

QVector QVector::operator-(QVector v1)
{
	QVector v2;
	v2.x = this->x - v1.x;
	v2.y = this->y - v1.y;
	v2.z = this->z - v1.z;
	return v2;
}

QVector QVector::operator*(float val)
{
	QVector v2;
	v2.x = this->x * val;
	v2.y = this->y * val;
	v2.z = this->z * val;
	return v2;
}

QVector QVector::operator/(float val)
{
	QVector v2;
	v2.x = this->x/val;
	v2.y = this->y/val;
	v2.z = this->z/val;
	return v2;
}

void QVector::setXYZ(float _x, float _y, float _z)
{
	this->x = _x;
	this->y = _y;
	this->z = _z;
}
float QVector::getX()
{
	return this->x;
}
float QVector::getY()
{
	return this->y;
}
float QVector::getZ()
{
	return this->z;
}
// Returns the normalized (norm ~= 1) version of the argument vector
QVector QVector::unitVector()
{
	float nrm = this->magnitude();

	QVector vec(this->x,this->y,this->z);
	vec = vec * (1.0/nrm);

	return vec;
}

// Returns the norm of the input vector
float QVector::magnitude()
{
	return sqrt(pow(this->x,2) + pow(this->y,2) + pow(this->z,2));
}

QVector QVector::emptyVector()
{
	QVector vec;
	vec.setEmpty(true);
	return vec;
}

void QVector::setEmpty(bool _empty)
{
	empty = _empty;
}

bool QVector::isEmpty()
{
	return empty;
}

const char* QVector::toString()
{
	ostringstream os;

	os.setf(ios::fixed);
//	os << "x:" << x << " y:" << y << " z:" << z;
	
	// Temp fix.  Use the one above for the actual printing
	os << "x:";
	(x == -0 )? os << 0.0 : os << x;
	os << " y:";
	(y == -0 )? os << 0.0 : os << y;
	os << " z:";
	(z == -0 )? os << 0.0 : os << z;

	return os.str().c_str();	
}


QVector QVector::vectorCross(QVector V2)
{

	QVector V3;
	V3.x = (this->y*V2.z) - (this->z*V2.y);
	V3.y = (this->z*V2.x) - (this->x*V2.z);
	V3.z = (this->x*V2.y) - (this->y*V2.x);
	return V3;
}

float QVector::vectorDot(QVector V2)
{
	float myX = this->x*V2.x;
	float myY = this->y*V2.y;
	float myZ = this->z*V2.z;
	return (myX+myY+myZ);
}
