#ifndef KALMAN_H
#define KALMAN_H

#include "math.h"
#include "Matrix.h"

#define NUM_SENSORS	11

using namespace std;

class Kalman
{
public:
	Kalman( float t_sample, float init = 200.0, float process = 0.3, float measure = 0.7 );
	void filter(unsigned char myID, float &heading, float &pitch, float &roll);
	
private:
	Matrix A;
	Matrix C;
	Matrix P[NUM_SENSORS];
	Matrix Q;
	Matrix R;

	// Intermediate matrices
	Matrix innovation;
	Matrix S;
	Matrix K;
	Matrix I;
	
	Matrix x_pred;
	Matrix P_pred;
	
	Matrix p_tran_c;// Scratch matrix of P_pred * transpose(C)

	Matrix kalman_x[NUM_SENSORS];
	Matrix kalman_y[NUM_SENSORS];
	bool firstFilterUse[NUM_SENSORS];
};

#endif
