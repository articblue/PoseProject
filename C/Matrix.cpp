#include "Matrix.h"

using namespace std;

// Default constructor
Matrix::Matrix()
{
	rows = 0;
	cols = 0;
	data = NULL;
}

// Matrix constructor with row r, column c, and initial data d
Matrix::Matrix(int r, int c, float** d)
{
	rows = r;
	cols = c;
	data = new float* [r];
	for(int i=0; i<r; i++)
		data[i] = new float[c];

	for(int i=0; i<r; i++)
		for(int j=0; j<c; j++)
			data[i][j] = d[i][j];
}

Matrix::Matrix(int r, int c, float d)
{
	rows = r;
	cols = c;
		data = new float* [r];
	for(int i=0; i<r; i++)
		data[i] = new float[c];

	for(int i=0; i<r; i++)
		for(int j=0; j<c; j++)
			data[i][j] = d;
}

Matrix::Matrix(int r, int c)
{
	rows = r;
	cols = c;
		data = new float* [r];
	for(int i=0; i<r; i++)
		data[i] = new float[c];

	for(int i=0; i<r; i++)
		for(int j=0; j<c; j++)
			data[i][j] = 0.0;
}

// destructor
Matrix::~Matrix()
{
	if ( data != NULL && data != 0)
	{
		for(int i=0; i<rows; i++)
			delete[] data[i];
		delete[] data;
	}
}

void Matrix::setDimensions(int r, int c)
{
	int i, j;

	rows = r;
	cols = c;
		data = new float* [r];
	for(i = 0; i < r; i++)
		data[i] = new float[c];

	for(i = 0; i < r; i++)
		for(j = 0; j < c; j++)
			data[i][j] = 0.0;
}

// Matrix operator overloads below
Matrix Matrix::operator*(const Matrix &m)
{
	int i, j, k;

	Matrix product(rows, m.cols);

	if(cols != m.rows)
	{
		//return m;
		printf("Your program is f___ed because %d not equal to %d\n.", rows, m.cols);	
		return product;
	}

	float index_sum;

	for(i=0; i<product.rows; i++)
	{
		for(j=0; j<product.cols; j++)
		{
			index_sum = 0.0;
			for(k=0; k<cols; k++)
				index_sum += data[i][k] * m.data[k][j];
			product.data[i][j] = index_sum;
		}
	}

	return product;
}

Matrix Matrix::operator+(const Matrix &m)
{
	int i, j;

	Matrix sum(m.rows, m.cols);

	for(i = 0; i<rows; i++)
		for(j = 0; j<cols; j++)
			sum.data[i][j] = data[i][j] + m.data[i][j];

	return sum;
}

Matrix Matrix::operator-(const Matrix &m)
{
	int i, j;

	Matrix diff(m.rows, m.cols);

	for(i = 0; i<rows; i++)
		for(j = 0; j<cols; j++)
			diff.data[i][j] = data[i][j] - m.data[i][j];

	return diff;
}

Matrix& Matrix::operator=(const Matrix &m)
{
	int i, j;

	rows = m.rows;
	cols = m.cols;
	for(i = 0; i < rows; i++)
		for(j = 0; j < cols; j++)
			data[i][j] = m.data[i][j];
	return *this;
}


Matrix negativize(const Matrix &m)
{
	int i, j;

	Matrix ans(m.rows, m.cols);

	for(i = 0; i < m.rows; i++)
		for(j = 0; j < m.cols; j++)
			ans.data[i][j] = m.data[i][j] * -1;
	return ans;
}

Matrix transpose(Matrix &m)
{
	int i, j;

	Matrix m_transpose(m.cols, m.rows);

	for(i = 0; i < m.rows; i++)
		for(j = 0; j < m.cols; j++)
			m_transpose.data[j][i] = m.data[i][j];

	return m_transpose;
}

// calculate the cofactor of element (row,col)
int getMinor(float **src, float **dest, int row, int col, int order)
{
    // indicate which col and row is being copied to dest
    int colCount=0,rowCount=0;

    for(int i = 0; i < order; i++ )
    {
        if( i != row )
        {
            colCount = 0;
            for(int j = 0; j < order; j++ )
            {
                // when j is not the element
                if( j != col )
                {
                    dest[rowCount][colCount] = src[i][j];
                    colCount++;
                }
            }
            rowCount++;
        }
    }

    return 1;
}

// Calculate the determinant recursively.
double calcDeterminant( float **mat, int order)
{
    // order must be >= 0
	// stop the recursion when matrix is a single element
    if( order == 1 )
        return mat[0][0];

    // the determinant value
    float det = 0;

    // allocate the cofactor matrix
    float **minor;
    minor = new float*[order-1];
    for(int i=0;i<order-1;i++)
        minor[i] = new float[order-1];

    for(int i = 0; i < order; i++ )
    {
        // get minor of element (0,i)
        getMinor( mat, minor, 0, i , order);
        // the recusion is here!
        det += pow( -1.0, i ) * mat[0][i] * calcDeterminant( minor,order-1 );
    }

    // release memory
    for(int i=0;i<order-1;i++)
        delete [] minor[i];
    delete [] minor;

    return det;
}

// matrix inversioon
// the result is put in Y
void matrixInversion(float **A, int order, float **Y)
{
    // get the determinant of a
    double det = 1.0/calcDeterminant(A,order);

    // memory allocation
    float *temp = new float[(order-1)*(order-1)];
    float **minor = new float*[order-1];
    for(int i=0;i<order-1;i++)
        minor[i] = temp+(i*(order-1));

    for(int j=0;j<order;j++)
    {
        for(int i=0;i<order;i++)
        {
            // get the co-factor (matrix) of A(j,i)
            getMinor(A,minor,j,i,order);
            Y[i][j] = det*calcDeterminant(minor,order-1);
            if( (i+j)%2 == 1)
                Y[i][j] = -Y[i][j];
        }
    }

    // release memory
    delete [] minor[0];
    delete [] minor;
}
	
Matrix inverse(const Matrix &m)
{
	Matrix input(m.rows, m.cols), output(m.rows, m.cols);
	input = m;
	matrixInversion(input.data, m.rows, output.data);
	return output;
}

Matrix inverse_3(const Matrix &m)
{
	Matrix result(3, 3);
	
	float det = 0.0 + m.data[0][0] * (m.data[1][1] * m.data[2][2] - m.data[2][1] * m.data[1][2])
			 - m.data[0][1] * (m.data[1][0] * m.data[2][2] - m.data[1][2] * m.data[2][0])
			 + m.data[0][2] * (m.data[1][0] * m.data[2][1] - m.data[1][1] * m.data[2][0]);

	float det_inv = 1.0 / det;

	result.data[0][0] =  (m.data[1][1]*m.data[2][2]-m.data[2][1]*m.data[1][2])*det_inv;
	result.data[0][1] = -1.0*(m.data[0][1]*m.data[2][2]-m.data[0][2]*m.data[2][1])*det_inv;
	result.data[0][2] =  (m.data[0][1]*m.data[1][2]-m.data[0][2]*m.data[1][1])*det_inv;
	result.data[1][0] = -1.0*(m.data[1][0]*m.data[2][2]-m.data[1][2]*m.data[2][0])*det_inv;
	result.data[1][1] =  (m.data[0][0]*m.data[2][2]-m.data[0][2]*m.data[2][0])*det_inv;
	result.data[1][2] = -1.0*(m.data[0][0]*m.data[1][2]-m.data[1][0]*m.data[0][2])*det_inv;
	result.data[2][0] =  (m.data[1][0]*m.data[2][1]-m.data[2][0]*m.data[1][1])*det_inv;
	result.data[2][1] = -1.0*(m.data[0][0]*m.data[2][1]-m.data[2][0]*m.data[0][1])*det_inv;
	result.data[2][2] =  (m.data[0][0]*m.data[1][1]-m.data[1][0]*m.data[0][1])*det_inv;

	return result;
}
