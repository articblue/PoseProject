#include "AngleContainer.h"

AngleContainer::AngleContainer()
{
		myID = -1;
		xC =0;
		yC = 0;
		zC = 0;
	
		xG = 0;
		yG = 0;
		zG = 0;
		
		xA = 0;
		yA = 0;
		zA = 0;
}

AngleContainer::AngleContainer(unsigned char id)
{
		myID = id;
		xC =0;
		yC = 0;
		zC = 0;
	
		xG = 0;
		yG = 0;
		zG = 0;
		
		xA = 0;
		yA = 0;
		zA = 0;
}


AngleContainer *  AngleContainer::getAngles()
{
	return this;
}

void AngleContainer::setAngles(float x, float y, float z, int ANGLETYPE)
{
	if (ANGLETYPE == ANGLE_COMPASS)
	{
		xC =x;
		yC = y;
		zC = z;
	}
	if (ANGLETYPE == ANGLE_GYRO)
	{
		xG = x;
		yG = y;
		zG = z;
	}
	if (ANGLETYPE == ANGLE_ACCEL)
	{
		xA = x;
		yA = y;
		zA = z;
	}
	
}
