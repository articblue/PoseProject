#include "Kalman.h"

static const float identity[6][6] = { {1, 0, 0, 0, 0, 0}, 
					{0, 1, 0, 0, 0, 0}, 
					{0, 0, 1, 0, 0, 0}, 
					{0, 0, 0, 1, 0, 0}, 
					{0, 0, 0, 0, 1, 0}, 
					{0, 0, 0, 0, 0, 1} };

static const float identity_3[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

float kalman_A[6][6] = { {1, 0, 0, 999, 0, 0},
					{0, 1, 0, 0, 999, 0},
					{0, 0, 1, 0, 0, 999},
					{0, 0, 0, 1, 0, 0},
					{0, 0, 0, 0, 1, 0},
					{0, 0, 0, 0, 0, 1} };

static const float kalman_C[3][6] = { {1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0} };

Kalman::Kalman( float t_sample, float init, float process, float measure )
{
	float P_init = init;
	float process_noise = process;
	float meas_noise = measure;

	// Set T_SAMPLE
	kalman_A[0][3] = kalman_A[1][4] = kalman_A[2][5] = t_sample;
	
	A.setDimensions(6, 6);
	C.setDimensions(3, 6);
	Q.setDimensions(6, 6);
	R.setDimensions(3, 3);
	
	// Intermediate matrices
	innovation.setDimensions(3, 1);
	S.setDimensions(3, 3);
	K.setDimensions(6, 3);
	I.setDimensions(6, 6);
	
	x_pred.setDimensions(6, 1);
	P_pred.setDimensions(6, 6);

	p_tran_c.setDimensions(6, 3);

	// Init the Kalman filter
	for(int k=0; k < NUM_SENSORS; k++)
	{
		P[k].setDimensions(6, 6);
		kalman_x[k].setDimensions(6, 1);
		kalman_y[k].setDimensions(3, 1);
		firstFilterUse[k] = true;
			
		for(int i = 0; i < 6; i++)
		{
			for(int j = 0; j < 6; j++)
			{
				P[k].data[i][j] = identity[i][j] * P_init;
				
				if(k == 0)
				{
					A.data[i][j] = kalman_A[i][j];
					Q.data[i][j] = identity[i][j] * process_noise * process_noise;
					I.data[i][j] = identity[i][j];
				}
			}
		}
	}

	for(int i=0; i < 3; i++)
		for(int j=0; j < 6; j++)
			C.data[i][j] = kalman_C[i][j];

	for(int i=0; i < 3; i++)
		for(int j=0; j < 3; j++)
			R.data[i][j] = identity_3[i][j] * meas_noise * meas_noise;
}

void Kalman::filter(unsigned char myID, float &heading, float &pitch, float &roll)
{
	// It's time to get filter.
	kalman_y[myID].data[0][0] = heading;
	kalman_y[myID].data[1][0] = pitch;
	kalman_y[myID].data[2][0] = roll;

	if(firstFilterUse[myID])
	{
		kalman_x[myID].data[0][0] = heading;
		kalman_x[myID].data[1][0] = pitch;
		kalman_x[myID].data[2][0] = roll;
		kalman_x[myID].data[3][0] = 0.0;
		kalman_x[myID].data[4][0] = 0.0;
		kalman_x[myID].data[5][0] = 0.0;
		firstFilterUse[myID] = false;
	}

	// 360 degree adjustment
	if(heading - kalman_x[myID].data[0][0] > 330.0)
		kalman_x[myID].data[0][0] += 360.0;
	if(heading - kalman_x[myID].data[0][0] < -330.0)
		kalman_x[myID].data[0][0] -= 360.0;

	if(pitch - kalman_x[myID].data[1][0] > 160.0)
		kalman_x[myID].data[1][0] += 180.0;
	if(pitch - kalman_x[myID].data[1][0] < -160.0)
		kalman_x[myID].data[1][0] -= 180.0;

	if(roll - kalman_x[myID].data[2][0] > 330.0)
		kalman_x[myID].data[2][0] += 360.0;
	if(roll - kalman_x[myID].data[2][0] < -330.0)
		kalman_x[myID].data[2][0] -= 360.0;
	
	// Stage 1:  Predict
	// x_pred = A * x
	x_pred = A * kalman_x[myID];

	// P_pred = A * P * A' + Q
	P_pred = (A * P[myID] * transpose(A)) + Q;

	// Stage 2:  Update

	// y_residual = y - C * x_pred
	innovation = kalman_y[myID] - (C * x_pred);

	// Scratch matrix, used to cut down on computation below
	p_tran_c = P_pred * transpose(C);

	// S = C * P_pred * C' + R
	S = (C * p_tran_c) + R;

	// K = P_pred * C' * inv(S)
	K = p_tran_c * inverse_3(S);

	// x = x_pred + K * y_residual
	kalman_x[myID] = x_pred + (K * innovation);

	// P = (I - K * C) * P_pred
	P[myID] = (I - (K * C)) * P_pred;

	// Print out top part of x here.
	heading = kalman_x[myID].data[0][0];
	pitch = kalman_x[myID].data[1][0];
	roll = kalman_x[myID].data[2][0];
}

