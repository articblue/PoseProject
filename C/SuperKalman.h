#ifndef SUPERKALMAN_H
#define SUPERKALMAN_H

#include "math.h"
#include "Matrix.h"
#include "Quaternion.h"

#define DT 0.016404199
#define NORMALIZE_FREQ 5

using namespace std;

class SuperKalman
{

public:
	SuperKalman();
	void update(Quaternion&, QVector &w, const QVector &a, bool has_compass, const int &g);
	
private:
	/*
	Matrix A;		// state transition matrix
	Matrix H;		// observational model
	Matrix xhat_;	// a priori state estimate
	Matrix xhat;	// a posteriori state estimate
	Matrix P_;		// a priori estate error covariance
	Matrix P;		// a posteriori estate error covariance
	Matrix K;		// kalman gain
	Matrix E;		// process noise covariance
	Matrix R;		// measurement noise covariance
	Matrix I_7;		// 7x7 identity matrix

	Matrix z;	

	float pc_q;
	float pc_g;
	
	float mc_q;
	float mc_g;
	*/

	Quaternion qc;
	QVector wp;
	int sample_ct;
	
};

#endif
