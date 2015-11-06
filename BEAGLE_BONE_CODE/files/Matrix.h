#ifndef MATRIX_H
#define MATRIX_H

#include <cstdio>
#include <math.h>
#include <string>
#include <sstream>

using namespace std;

class Matrix
{
public:
	Matrix();
//	Matrix(Matrix );
	Matrix(int, int, float**);
	Matrix(int, int, float);
	Matrix(int, int);

	~Matrix();

	void setDimensions(int r, int c);

	Matrix operator*(const Matrix&);
	Matrix operator+(const Matrix&);
	Matrix operator-(const Matrix&);
	Matrix& operator=(const Matrix&);
	friend void matrixInversion(float **A, int order, float **Y);
	friend int getMinor(float **src, float **dest, int row, int col, int order);
	friend double calcDeterminant( float **mat, int order);


	int rows;
	int cols;
	
	float **data;

	friend Matrix transpose(Matrix&);
	friend Matrix negativize(const Matrix&);
	friend Matrix inverse(const Matrix&);
	friend Matrix inverse_3(const Matrix&);
};


#endif
