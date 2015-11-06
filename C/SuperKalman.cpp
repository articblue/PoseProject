#include "SuperKalman.h"

SuperKalman::SuperKalman()
{
	sample_ct = 0;
	wp.x = wp.y = wp.z = 0.0;
	
	/*
	// noise constants...
	pc_q = 0.5;
	pc_g = 1.570796327; // pi/2
	mc_q = 10.0; // updated dynamically by filter
	mc_g = 0.0005;
	
	// state transition matrix
	A.setDimensions(7,7);
	for (int i = 0; i < 7; i++)
		A.data[i][i] = 1.0;
	
	// observation model
	H.setDimensions(7,7);
	for (int i = 0; i < 7; i++)
		H.data[i][i] = 1.0;
	
	// state estimates
	xhat_.setDimensions(7,1);
	xhat.setDimensions(7,1);
	
	// estimate error covariance matrices
	P_.setDimensions(7,7);
	P.setDimensions(7,7);
	
	// kalman gain
	K.setDimensions(7,7);
	
	// declare and init process noise matrix
	E.setDimensions(7,7);
	for (int i = 0; i < 4; i++) E.data[i][i] = pc_q;
	for (int i = 4; i < 7; i++) E.data[i][i] = pc_g;
	
	// declare and init measurement noise matrix
	R.setDimensions(7,7);
	for (int i = 0; i < 4; i++) R.data[i][i] = mc_q;
	for (int i = 4; i < 7; i++) R.data[i][i] = mc_g;
	
	// declare and init 7x7 identity matrix
	I_7.setDimensions(7,7);
	for (int i = 0; i < 7; i++)
		I_7.data[i][i] = 1.0;
	
	// Measurement vector
	z.setDimensions(7,1);

	//
	// initial state and estimate error covariance
	//
	
	xhat.data[0][0] = 1.0;
	xhat.data[1][0] = 0.0;
	xhat.data[2][0] = 0.0;
	xhat.data[3][0] = 0.0;
	xhat.data[4][0] = 0.0;
	xhat.data[5][0] = 0.0;
	xhat.data[6][0] = 0.0;
	
	for (int i = 0; i < 7; i++)
		P.data[i][i] = 100.0;
	*/
}

void SuperKalman::update(Quaternion &q, QVector &w, const QVector &a, bool has_compass, const int &g)
{
	Quaternion qtemp;

	if ( sample_ct == 0 && !(has_compass) )
		return;

/*	if ( sample_ct == 0 || (sample_ct == 15 && has_compass) )
	{
		wp.x = 0;
		wp.y = 0;
		wp.z = 0;
		qc = q;
	}
	else
	{
		wp.x = qc.toVector().x;
		wp.y = qc.toVector().y;
		wp.z = qc.toVector().z;

		//wp.x = g * (3.1415927/180.0);

		float y_pitch_mul, z_heading_mul, y_heading_mul, z_pitch_mul;
		float pitchContribution, headingContribution;
                // Compute signs for the pitch and heading contributions^M
                // These signs depend on the roll (orientation) of the board in space.  (Proof of this concept left to the reader.)^M
                y_pitch_mul = (cos(wp.x) > 0.0) ? 1.0 : -1.0;
                z_heading_mul = y_pitch_mul;

                y_heading_mul = (sin(wp.x) > 0.0) ? 1.0 : -1.0;
                z_pitch_mul = -y_heading_mul;

                // Compute heading and pitch contributions from roll angle and gyro readings
                pitchContribution = y_pitch_mul * w.y * pow(cos(wp.x), 2) + z_pitch_mul * w.z * pow(sin(wp.x), 2);
                headingContribution = y_heading_mul * w.y * pow(sin(wp.x), 2) + z_heading_mul * w.z * pow(cos(wp.x), 2);

		wp.x = w.x; // roll always matches
		wp.y = pitchContribution;
		wp.z = headingContribution;

                if( wp.y > 90.0 || wp.y < -90.0 )
                {
                        // Implement flip algorithm
                        wp.x += 180.0;
                        wp.y = (wp.y > 90.0)? 180.0 - wp.y : -180.0 - wp.y;
                        wp.z += 180.0;
                }

                // Fix heading and roll
                while(wp.z > 360.0)
                        wp.z -= 360.0;
                while(wp.x > 360.0)
                        wp.x -= 360.0;


	}

	if ( has_compass ) 
		sample_ct = ((sample_ct) % 15) + 1;//(sample_ct + 1) % 6;

	Quaternion dp(wp.x, wp.y, wp.z);

	qc = qc * dp; // Global * Local = Global

	qc.normalize();

	q = qc;*/


	sample_ct = 1;

	// THIS PAGE EXPLAINS WHY THE BELOW CODE WORKS: http://www.euclideanspace.com/physics/kinematics/angularvelocity/
	// Converts Gyro so it can be multiplied by the compass
	QVector wavg = w;	// average angular velocity across ( now-DT, now )

	Quaternion dq(0, wavg);		// quaternion for angular velocity

	qtemp = q * dq;

	qtemp.w        *= DT/2;
	qtemp.vector.x *= DT/2;
	qtemp.vector.y *= DT/2;
	qtemp.vector.z *= DT/2;

	qc.w        = q.w        + qtemp.w;
	qc.vector.x = q.vector.x + qtemp.vector.x;
	qc.vector.y = q.vector.y + qtemp.vector.y;
	qc.vector.z = q.vector.z + qtemp.vector.z;

	qc.normalize();

	q = qc;

// Compass + Gyro
/*	if ( sample_ct == 0 && !(has_compass) )
		return;


	if (sample_ct == 0 )
	{
		qc = q;
	}
	else
	{
		QVector wavg = (wp + w);	// average angular velocity across ( now-DT, now )
		wavg.x /= 2.0;
		wavg.y /= 2.0;
		wavg.z /= 2.0;

		Quaternion dq(0, wavg);		// quaternion for angular velocity

		Quaternion qtemp;
		qtemp = qc * dq;// * qc;

		qtemp.w        *= DT/2;
		qtemp.vector.x *= DT/2;
		qtemp.vector.y *= DT/2;
		qtemp.vector.z *= DT/2;

		qc.w        = qc.w        + qtemp.w;
		qc.vector.x = qc.vector.x + qtemp.vector.x;
		qc.vector.y = qc.vector.y + qtemp.vector.y;
		qc.vector.z = qc.vector.z + qtemp.vector.z;

		if (sample_ct % ((int)(1/DT)/NORMALIZE_FREQ) == 0)
		{
			qc.normalize();
		}
	}
	sample_ct++;
	q = qc;*/


// Extended Kalman Filter
/*	float wx, wy, wz, q1, q2, q3, q4;
	float dtwx, dtwy, dtwz, dtq1, dtq2, dtq3, dtq4;
	float qmag, amag;



	// // // // // // // // // // // // // // // // //
	//                                              //
	//   Predict                                    //
	//                                              //
	// // // // // // // // // // // // // // // // //
	
	
	// put last a posteriori state estimate into more friendly variables
	q1 = xhat.data[0][0];
	q2 = xhat.data[1][0];
	q3 = xhat.data[2][0];
	q4 = xhat.data[3][0];
	wx = xhat.data[4][0];
	wy = xhat.data[5][0];
	wz = xhat.data[6][0];
	
	
	// update state transition matrix
	// 
	// A = [
	// 		1	-dtwx	-dtwy	-dtwz	-dtq2	-dtq3	-dtq4
	// 	 dtwx		1	-dtwz	 dtwy	 dtq1	 dtq4	-dtq3
	// 	 dtwy	 dtwz		1	-dtwx	-dtq4	 dtq1	 dtq2
	// 	 dtwz	-dtwy	 dtwx		1	 dtq3	-dtq2	 dtq1
	// 		0		0		0		0		1		0		0
	// 		0		0		0		0		0		1		0
	// 		0		0		0		0		0		0		1
	// ]
	
	dtwx = DT*wx/2;
	dtwy = DT*wy/2;
	dtwz = DT*wz/2;
	dtq1 = DT*q1/2;
	dtq2 = DT*q2/2;
	dtq3 = DT*q3/2;
	dtq4 = DT*q4/2;
	
//	A.data[0][0] =     1;
	A.data[0][1] = -dtwx;
	A.data[0][2] = -dtwy;
	A.data[0][3] = -dtwz;
	A.data[0][4] = -dtq2;
	A.data[0][5] = -dtq3;
	A.data[0][6] = -dtq4;
	
	A.data[1][0] =  dtwx;
//	A.data[1][1] =     1;
	A.data[1][2] = -dtwz;
	A.data[1][3] =  dtwy;
	A.data[1][4] =  dtq1;
	A.data[1][5] =  dtq4;
	A.data[1][6] = -dtq3;
	
	A.data[2][0] =  dtwy;
	A.data[2][1] =  dtwz;
//	A.data[2][2] =     1;
	A.data[2][3] = -dtwx;
	A.data[2][4] = -dtq4;
	A.data[2][5] =  dtq1;
	A.data[2][6] =  dtq2;
	
	A.data[3][0] =  dtwz;
	A.data[3][1] = -dtwy;
	A.data[3][2] =  dtwx;
//	A.data[3][3] =     1;
	A.data[3][4] =  dtq3;
	A.data[3][5] = -dtq2;
	A.data[3][6] =  dtq1;

	wavg.x /= 2.0;
	wavg.y /= 2.0;
	wavg.z /= 2.0;
	
	// a priori state estimate
	xhat_.data[0][0] = q1 - DT/2*(q2*wx + q3*wy + q4*wz);
	xhat_.data[1][0] = q2 + DT/2*(q1*wx - q3*wz + q4*wy);
	xhat_.data[2][0] = q3 + DT/2*(q1*wy + q2*wz - q4*wx);
	xhat_.data[3][0] = q4 + DT/2*(q1*wz - q2*wy + q3*wx);
	xhat_.data[4][0] = wx;
	xhat_.data[5][0] = wy;
	xhat_.data[6][0] = wz;
	
	// normalize quaternion
	qmag = sqrt( pow(xhat_.data[0][0], 2) + pow(xhat_.data[1][0], 2) + pow(xhat_.data[2][0], 2) + pow(xhat_.data[3][0], 2) );
	xhat_.data[0][0] = xhat_.data[0][0] / qmag;
	xhat_.data[1][0] = xhat_.data[1][0] / qmag;
	xhat_.data[2][0] = xhat_.data[2][0] / qmag;
	xhat_.data[3][0] = xhat_.data[3][0] / qmag;
	
	// a priori estimate error covariance
	P_ = A * P * transpose(A) + E;
	


	// // // // // // // // // // // // // // // // //
	//                                              //
	//   Update                                     //
	//                                              //
	// // // // // // // // // // // // // // // // //
	
	
	// update measurement noise covariance matrix
	amag = sqrt( pow(a.x, 2) + pow(a.y, 2) + pow(a.z, 2) );
	mc_q = 10.0 * ( 1 + 10 * fabs( 1 - amag ) );
	for (int i = 0; i < 4; i++) R.data[i][i] = mc_q;
	
	// build measurement vector
	z.data[0][0] = q.w;
	z.data[1][0] = q.vector.x;
	z.data[2][0] = q.vector.y;
	z.data[3][0] = q.vector.z;
	z.data[4][0] = w.x;
	z.data[5][0] = w.y;
	z.data[6][0] = w.z;
	
	// determine kalman gain
	

	K = P_ * transpose(H) * inverse(H * P_ * transpose(H) + R);



	// a posteriori state estimate
	xhat = xhat_ + K * (z - H * xhat_);
	

	// a posteriori estimate error covariance
	P = ( I_7 - K * H ) * P_;
	
	// normalize quaternion
	qmag = sqrt( pow(xhat.data[0][0], 2) + pow(xhat.data[1][0], 2) + pow(xhat.data[2][0], 2) + pow(xhat.data[3][0], 2) );
	xhat.data[0][0] = xhat.data[0][0] / qmag;
	xhat.data[1][0] = xhat.data[1][0] / qmag;
	xhat.data[2][0] = xhat.data[2][0] / qmag;
	xhat.data[3][0] = xhat.data[3][0] / qmag;
	
	// return filtered state
	q.w        = xhat.data[0][0];
	q.vector.x = xhat.data[1][0];
	q.vector.y = xhat.data[2][0];
	q.vector.z = xhat.data[3][0];
	w.x	   = xhat.data[4][0];
	w.y	   = xhat.data[5][0];
	w.z	   = xhat.data[6][0];
*/
}

