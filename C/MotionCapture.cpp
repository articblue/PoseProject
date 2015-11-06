
#include "MotionCapture.h"

static const float accel_sensitivity_x[] = { 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038};
static const float accel_sensitivity_y[] = { 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0038, 0.0037, 0.0038, 0.0038};
static const float accel_sensitivity_z[] = { 0.0039, 0.0039, 0.0039, 0.004, 0.004, 0.004, 0.004, 0.0039, 0.0039, 0.004};
static const float accel_calibration_x[] = { 0.025, -0.025, -0.025, -0.035, -0.04, -0.035, -0.04, -0.03, -0.05, -0.02};
static const float accel_calibration_y[] = { -0.015, 0.01, -0.005, -0.005, 0.03, -0.005, -0.02, 0.025, -0.03, -0.02 };
static const float accel_calibration_z[] = { -0.05, 0.01, -0.02, 0.03, 0.085, 0.03, 0.06, 0.015, -0.05, 0.1 };

MotionCapture::MotionCapture()
{
	// Vector representation of the body model
	// each limb is an upper / lower segment
	// therefore each person has a left/right arm/leg
	TotalBuffSize = 0;
	QVector tempVectRU(1,0,0);
	QVector tempVectLU(-1,0,0);
	QVector tempVectRL(2,0,0);
	QVector tempVectLL(-2,0,0);
	//QVector tempVectTORSO(-1,1,-1);
	QVector tempVectTORSO(0,1,0);
	fflush(stdout);
	string filename = "defaultOutput.bin";
	const char* myname = filename.c_str();
	read_from_serial = 1; // default to reading from serial and not file
	myOutputStream.open(myname,ios::out|ios::binary);

	
	serialOutput = STDOUT_FILENO;
	if ( serialOutput == -1 ) 
		fprintf(stdout, "ERROR\n");

	for ( int i = 0; i < NUM_SENSORS; i++ )
	{
		gyroX[i] = 0;
		gyroY[i] = 0;
		gyroZ[i] = 0;
		counter[i] = 0;

		gyro_calibration_x[i] = 0;
		gyro_calibration_y[i] = 0;
		gyro_calibration_z[i] = 0;
	}

	// Create Kalman classes
	kalman_level = new Kalman(T_SAMPLE, 200.0, 0.3, 0.7);
	kalman_upright = new Kalman(T_SAMPLE, 200.0, 0.3, 0.7);
	
	// Generatre the quaternions necessary to
	// transform the jumpsuit system from a local
	// quaternion representation to a reference system that is based on teh TORSO
	
	// To transform the system after the TORSO reference, the whole system will need
	// to be rotated by the quaternion that represents the rotation from the TORSO
	// quaternion to the global reference 'pure' quaternion ie( a vector with a 0 w)
	// Maybe add this in to allow the system to be dynamically rotated to the
	// global reference system
	ReferenceOrientationQuat[RU_ARM] = Quaternion(0,tempVectRU);
	ReferenceOrientationQuat[RL_ARM] = Quaternion(0,tempVectRL);
	ReferenceOrientationQuat[RU_LEG] = Quaternion(0,tempVectRU);
	ReferenceOrientationQuat[RL_LEG] = Quaternion(0,tempVectRL);
	ReferenceOrientationQuat[LU_ARM] = Quaternion(0,tempVectLU);
	ReferenceOrientationQuat[LL_ARM] = Quaternion(0,tempVectLL);
	ReferenceOrientationQuat[LU_LEG] = Quaternion(0,tempVectLU);
	ReferenceOrientationQuat[LL_LEG] = Quaternion(0,tempVectLL);
	ReferenceOrientationQuat[TORSO] = Quaternion(0,tempVectTORSO);
	// Normalize the quaternions, this is necessary to
	// allow us to use the 'sandwitch' method to multiply / rotate the quaternions 
	//ReferenceOrientationQuat[RU_ARM].normalize();
	//ReferenceOrientationQuat[RL_ARM].normalize();
	//ReferenceOrientationQuat[RU_LEG].normalize();
	//ReferenceOrientationQuat[RL_LEG].normalize();
	ReferenceOrientationQuat[LU_ARM].normalize();
	//ReferenceOrientationQuat[LL_ARM].normalize();
	//ReferenceOrientationQuat[LU_LEG].normalize();
	//ReferenceOrientationQuat[LL_LEG].normalize();
	ReferenceOrientationQuat[TORSO].normalize();
	CompassVect[RU_ARM] = QVector();
	CompassVect[RL_ARM] = QVector();
	CompassVect[LU_ARM] = QVector();
	CompassVect[LL_ARM] = QVector();
	CompassVect[RU_LEG] = QVector();
	CompassVect[RL_LEG] = QVector();
	CompassVect[LU_LEG] = QVector();
	CompassVect[LL_LEG] = QVector();
	CompassVect[TORSO] = QVector();
	TORSO_FLAG = false;
	LU_ARM_AVG_IND = 0;
	LU_ARM_AVG[0] = QVector(0,0,0);
	LU_ARM_AVG[1] = QVector(0,0,0);
	LU_ARM_AVG[2] = QVector(0,0,0);
	debug_Counter = 0;
	for (int i=0;i<NUM_SENSORS;i++)
	{
		BaselineOffsetSet[i] = false;
		BaselineOffsetPoint[i] = QVector(0,0,0);
	}
	WaitCounter = 0;
}

int MotionCapture::openFile(char *name)
{
	inputFile = fopen(name, "r");

	if ( inputFile == NULL )
	{
	      perror("Error opening file");
	      return -1;
	}

	read_from_serial = 0;

	return 1;
}

void MotionCapture::closeFile()
{
	fclose(inputFile);
}

// Read the calibration file for the gyroscopes
bool MotionCapture::readConfigurationFile()
{
        int val;

        FILE *confFile = fopen("mocap.conf", "r");

        if ( confFile == NULL )
                return false;

        for ( int i = 0; i < 3; i++ ) // X = 0, Y = 1, Z = 2
        {
                for ( int j = 0; j < NUM_SENSORS; j++ )
                {
                        fscanf(confFile, "%d", &val);

                        switch (i)
                        {
                                case 0: gyro_calibration_x[j] = val; break;
                                case 1: gyro_calibration_y[j] = val; break;
                                case 2: gyro_calibration_z[j] = val; break;
                        }
                }
        }

        fclose(confFile);

        return true;
}

// Open the serial port 'path' usually /dev/ttyxxX
int MotionCapture::openSerialPort(char* path, bool read)
{
	struct termios options;
	struct termios gOriginalTTYAttrs;
	int serial;

	serial = open( path, O_RDWR | O_NOCTTY);

	fflush(stdout);

	if ( serial == -1 )
	{
		printf( "Error %d opening serial port %s: %s.\n", errno, path, strerror(errno)  );
		return (-1);
	}

	do {
		if (fcntl( serial, F_SETFL, 0 ) == -1)
		{
			printf( "Error clearing O_NDELAY %s - %s(%d).\n", path , strerror(errno), errno);
			break;
		}

		//	     fcntl(serial, F_SETFL, FNDELAY);

		// Get the current options and save them for later reset
		if (tcgetattr(serial, &gOriginalTTYAttrs) == -1)
		{
			printf( "Error getting tty attributes %s - %s(%d).\n", path , strerror(errno), errno);
			break;
		}
		// Set raw input, one second timeout
		// These options are documented in the man page for termios
		// (in Terminal enter: man termios)
		bzero(&options, sizeof(options)); /* clear struct for new port settings */
		cfmakeraw(&options);

		options = gOriginalTTYAttrs;
		options.c_iflag = 0;
		options.c_lflag = 0;
		options.c_oflag = 0;
		options.c_cc[VTIME] = 0; // Time to wait for a read to go
		options.c_cc[VMIN] = 22; // Minumum number of chars to recieve for a valid read()
		//options.c_cflag |= CS8; // 8n1 bits enable

		//cfsetspeed(&options, B38400);
		//cfsetspeed(&options, B115200);
		cfsetspeed(&options, B230400);
		//cfsetspeed(&options, B460800);
		//cfsetspeed(&options,B500000);
		tcflush(serialDescriptor, TCIFLUSH);
		// Set the options
		if (tcsetattr(serial, TCSANOW, &options) == -1)
		{
			printf( "Error setting tty attributes %s - %s(%d).\n", path , strerror(errno), errno);
			break;
		}

		serialDescriptor = serial;
		return 1;
	} while (0);

	// Bad state
	return -1;
}

// This function reads from the serial port and returns how many bytes the read sensor has
// This reads the data into a static circular buffer
// TODO: make the buffer an instance of the object, instead of using the static method
// Possible reasons this wasn't done was to prevent the uncessary creation
// of a buffer if we are reading from a file.

int MotionCapture::readSerial(char *serialArray, unsigned char &packet_flags)
{
	int i, length, read_amt = 0;
	length = 0;
	int read_res = 0;
	unsigned char chksum;

//	static unsigned char serial_buf[buf_len];	// buffer for data from usart, is static
//	static unsigned char tmp_buf[buf_len];
	static unsigned int write_pos = 0, read_pos = 0;

	static bool fill_buffer = true;

	do // while is used so I can use the break; or continue; statement
	{
		// Fill buffer
		if (fill_buffer || (read_pos == write_pos)
				|| ((read_pos > write_pos) && (((buf_len - read_pos) + write_pos) < FULL_PACKET_SIZE))
				|| ((read_pos < write_pos) && ((write_pos - read_pos) < FULL_PACKET_SIZE)) )
		{
			//usleep(100);
			if ( read_pos > write_pos )
				length = read_pos - write_pos;
			else
				length = (buf_len - write_pos) + read_pos;
			
			// Read will read the VMIN or upto buf_len
			read_res = read(serialDescriptor, &tmp_buf[0], buf_len);
			//fprintf(stdout,"Read Serial Num: %i\n",read_res);
			TotalBuffSize += read_res;
			for (i = 0; i < read_res; i++)
				serial_buf[(write_pos + i) % buf_len] = tmp_buf[i];
			write_pos = (write_pos + read_res) % buf_len;
			
			fill_buffer = false; // done with buffer
			read_res = 0;

		}
		
		// Extract the data
		do
		{
			// set flags
			packet_flags = (serial_buf[read_pos] & ACCEL_FLAG) | (serial_buf[read_pos] & COMPASS_FLAG) | (serial_buf[read_pos] & ORIEN_FLAG);

			// Read ID
			if ( serial_buf[read_pos] & 0x10 )		  				     // Invalid
				read_amt = -1;
			else if ( serial_buf[read_pos] & ACCEL_FLAG && serial_buf[read_pos] & COMPASS_FLAG ) // 0b11000000 -- compass + accel
				read_amt = FULL_PACKET_SIZE;
			else if ( serial_buf[read_pos] & ACCEL_FLAG || serial_buf[read_pos] & COMPASS_FLAG ) // (compass + no accel) or (no compass + accel)
				read_amt = HALF_PACKET_SIZE;
			else										     // (no compass) and (no accel)
				read_amt = QUARTER_PACKET_SIZE;

			if ( read_amt == -1 ) // Invalid ID
			{
				read_pos = (read_pos + 1) % buf_len;
				//TotalBuffSize -=1;
				// need to refill buffer -- no more data
				if ((read_pos == write_pos)
					|| ((read_pos > write_pos) && (((buf_len - read_pos) + write_pos)) < FULL_PACKET_SIZE)
					|| (read_pos < write_pos) && (((write_pos - read_pos) < FULL_PACKET_SIZE)) )
				{
					fill_buffer = true;
					break;
				}

				continue; // read new ID
			}
			//fprintf(stdout,"I ATE %i bytes of the buffer\n",read_amt);
			//TotalBuffSize -= read_amt;

			// Verify CHKSUM
			chksum = 0x00;
			for ( i = read_pos; i < read_pos + read_amt; i++) // read all but chksum bit
				chksum += serial_buf[i % buf_len];

			if ( chksum != serial_buf[(read_pos + read_amt) % buf_len] )
			{
				read_pos = (read_pos + 1) % buf_len;

				// need to refill buffer -- no more data
				if (read_pos == write_pos
					|| (read_pos > write_pos) && (((buf_len - read_pos) + write_pos) < FULL_PACKET_SIZE)
					|| (read_pos < write_pos) && ((write_pos - read_pos) < FULL_PACKET_SIZE) )
				{
					fill_buffer = true;
					break;
				}

				continue; // read new ID
			}
			// END CHKSUM

			// Print data
			for ( i = read_pos; i < read_pos + read_amt; i++){ // only print to length, don't print checksum
				serialArray[i - read_pos] = serial_buf[i % buf_len];}

			// Advance read position
			read_pos = (read_pos + read_amt + 1) % buf_len; // + 1 for length instead of index
			//TotalBuffSize -= 1;
		} while (0);

		if ( fill_buffer){ // means we ran out of buffer to read from
			continue;}

	} while(0);

	return read_amt; // returns the amount of bytes it had to read
}

int MotionCapture::readFile(char *serialArray, unsigned char &packet_flags, const bool &file_encrypted)
{
	int c, i = 0;
	int length;

	// Reached end of file without a full packet
	if ( feof(inputFile) )
		return -1;

	// Read from file or stream
	if ( !file_encrypted )
		fscanf(inputFile, "%02x", &c);
	else
		c = fgetc(inputFile);

        packet_flags = (c & ACCEL_FLAG) | (c & COMPASS_FLAG) | (c & ORIEN_FLAG);

	if ( c & ACCEL_FLAG && c & COMPASS_FLAG )	// 0b11000000 -- compass + accel
		length = FULL_PACKET_SIZE;
	else if ( c & ACCEL_FLAG || c & COMPASS_FLAG )	// (compass + no accel) or (no compass + accel)
		length = HALF_PACKET_SIZE;
	else						// (no compass) and (no accel)
		length = QUARTER_PACKET_SIZE;

	serialArray[i++] = c;

	while ( i < length )
	{
		// Reached end of file without a full packet
		if ( feof(inputFile) )
			return -1;

		// Read from file or stream
		if ( !file_encrypted )
			fscanf(inputFile, "%02x", &c);
		else
			c = fgetc(inputFile);
		
		serialArray[i++] = c;
	}

	return length;
}

// Returns how long the sensor's packet was
// +1 is used for CHKSUM
void MotionCapture::getPacket(int &len, char packet[FULL_PACKET_SIZE_CHKSUM], unsigned char &packet_flags, const bool &file_encrypted)
{
	if ( read_from_serial )
	{
		len = readSerial(packet, packet_flags);
		TotalBuffSize -= (len+1);
		//fprintf(stdout,"LengthRead: %i TotalBuff Size: %i\n",len, TotalBuffSize);
		
	}
	else
		len = readFile(packet, packet_flags, file_encrypted);
}

// Returns the lower 4 bits of the first byte
char MotionCapture::getID(char* packet)
{
	return packet[0] & 0x0F;
}


// Returns true if the accelerometer is "still" and false otherwise;
// also, provides the raw accel data in numeric form
bool MotionCapture::check_still(char* packet, QVector &data)
{
	float x, y, z, nrm;
//	float stillMax = 1, stillMin = 2;

	// Regardless of packet type, accel data is in bytes 1 - 6
	// (Indices 0 - 5)

	// These can be floats maybe? see if shorter data type is needed
	x = hex_to_dec(packet[5], packet[6]) * 0.004;
	y = hex_to_dec(packet[7], packet[8]) * 0.004;
	z = hex_to_dec(packet[9], packet[10]) * 0.004;

	data.setXYZ(x, y, z);
	nrm = data.magnitude();

	// Look up "still value range" from some calibration table
	// (do that here)

//	return ( nrm < stillMax && nrm > stillMin ); 
	return true; // replace this
}

// Extract the accelerometer portion of the packet for the node ID specified.
// this is then used in a simple sanity check
// based on the 'old' value and update value
QVector MotionCapture::extractAccel(unsigned char myID, char* packet, float &pitch, float &roll)
{
	float accelx, accely, accelz;
//	float pi_over_180 = PI / 180.0;

	accelx = hex_to_dec(packet[5], packet[6]) * accel_sensitivity_x[myID] + accel_calibration_x[myID];
	accely = hex_to_dec(packet[7], packet[8]) * accel_sensitivity_y[myID] + accel_calibration_y[myID];
	accelz = hex_to_dec(packet[9], packet[10]) * accel_sensitivity_z[myID] + accel_calibration_z[myID];

	QVector vec;
	// Adding new qvectors to
	// take advantage of the class system
	// also to have a state system to fall back to
	// incase of invalid data for a accel update...
	 QVector * OLDV;
	vec.setXYZ(accelx, accely, accelz);

	// compare old vs new accel on x,y,z
	// if it's greater then some threshold
	// kill the update revert to old
	// else update the old and return that...
	// ASSUME that the first time is good...
	
	OLDV = getQvectorOld(myID);
	//fprintf(stdout,"DO WE GET HERE\n");	
	compareQVect(*OLDV,vec,.014);
	vec = *OLDV;
	setQvectorOld(vec,myID);
	OLDV = NULL;
	return vec;
}
// A simple comparitor
// Function: Compare the Left Hand Side, Right Hand Side based on a noise threashold
// If the result is below the noise threshold then we'll set it to the previous value
// Could be used to filter the accel instead of the Kalmann stuff
// This would only be necessary if the system is actually having a 'peaked' noise sprectrum on the accell data
// This is not known to be the case yet: meanign we haven't tested this to see

bool MotionCapture::compareQVect(QVector &LHS, QVector &RHS, float myThresh)
{
float Lx = LHS.x;
float Ly = LHS.y;
float Lz = LHS.z;
float Rx = RHS.x;
float Ry = RHS.y;
float Rz = RHS.z;

// first time through just
// set the rhs to the lhs and return
if(LHS.firstUpdate)
{
	LHS = RHS;
	return true;
}

// Looking at the noise in the system
if(abs(Lx - Rx)<= myThresh)
	Rx = LHS.x;
if(abs(Ly - Ry)<= myThresh)
	Ry = LHS.y;
if(abs(Lz - Rz)<= myThresh)
	Rz = LHS.z;

if(abs(Lx - Rx)> UPPERTHRESH)
	return false;
if(abs(Ly - Ry)> UPPERTHRESH)
	return false;
if(abs(Lz - Rz)> UPPERTHRESH)
	return false;

LHS.x = Rx;
LHS.y = Ry;
LHS.z = Rz;

return true;
}

QVector* MotionCapture::getQvectorOld(unsigned char myID)
{
	if(validID(myID))
		return  &OldAccelVect[myID];
	return NULL;
}

/*QVector* MotionCapture::getQvectorGyroOld(unsigned char myID)
{
	if(validID(myID))
		return  &OldGyroVect[myID];
	return NULL;
}
*/
void MotionCapture::setQvectorOld(QVector &RHS, unsigned char myID)
{
	OldAccelVect[myID] = RHS;
}

QVector MotionCapture::extractGyro(unsigned char myID, char* packet)
{
	float x, y, z, x_rot, y_rot, z_rot;
	char tmp;
	// Regardless of packet type, gyro data is in bytes 9 - 11
	// (Indices 8 - 10)

	// With only one byte, we can reuse conversion function by filling in 0
	// as most significant byte.

	// packet[4] structure -- 00XX YYZZ with MSb values

	// grab the x component of the byte
	tmp = (packet[4] & 0x30) >> 4;
	x = hex_to_dec(tmp, packet[1]) - GYRO_VREF;

	// grab the y component of the byte
	tmp = (packet[4] & 0x0C) >> 2;
	y = hex_to_dec(tmp, packet[2]) - GYRO_VREF;

	// grab the z component of the byte
	tmp = (packet[4] & 0x03);
	z = hex_to_dec(tmp, packet[3]) - GYRO_VREF; 

	// Gyro Calibrations
	x += gyro_calibration_x[myID];
	y += gyro_calibration_y[myID];
	z += gyro_calibration_z[myID];

	// Convert gyroscope raw data to radians per second.
	// This T_SAMPLE does not need to be here,
	// because SuperKalman does the needed division for me
	// Since we are NOT using the kalman filter stuff because it is 
	// broken. We will re-introduce the T_SAMPLE conversion
	x_rot = (x * GYRO_CONVERSION_CONST) *  T_SAMPLE * (PI / 180.0);
	y_rot = (y * GYRO_CONVERSION_CONST) *  T_SAMPLE * (PI / 180.0);
	z_rot = (z * GYRO_CONVERSION_CONST) *  T_SAMPLE * (PI / 180.0);

	// Adjusting this to the same as the compass orientation
	QVector vec(-z_rot, -y_rot, -x_rot);
	//if ( z_rot > PI )
	//	z_rot -= 2 * PI;

	//QVector vec(x_rot,y_rot,z_rot);
	
	return vec;
}

QVector MotionCapture::extractCompass(unsigned char myID, char* packet, int packet_size)
{
/*
 Function: extractCompass
 Extracts the packet data from the compass from degrees and converts 
 this over to radians.

 I hope radians are used throughout the system...
*/
	float heading, pitch, roll;
//	float heading2,pitch2,roll2;
//	float x, y, z;
	float pi_over_180 = PI / 180.0;
//	if ( packet_size == FULL_PACKET_SIZE )
	{
		heading = (hex_to_dec(packet[11], packet[12]) * 0.1)* pi_over_180;
		pitch = (hex_to_dec(packet[13], packet[14]) * 0.1) * pi_over_180;
		roll = (hex_to_dec(packet[15], packet[16]) * 0.1) *pi_over_180;
	}

	// Convert 0 to 2PI into -PI to PI for heading
	QVector vec2;
	//if ( heading > PI )
	//	heading -= 2 * PI;
	//if ( pitch > PI)
	//	pitch -= 2*PI;
	//if ( roll > PI)
	//	roll -= 2*PI;
	switch(myID)
	{
	case LU_ARM:{
		vec2.setXYZ(-roll,pitch,heading);
		//vec2.setXYZ(-heading,pitch,roll);
		orientationQuatLAG[LU_ARM] = orientationQuat[LU_ARM];
		
		// These are the localy defined axis on the boards
		X_Axis[LU_ARM].fromEuler(-roll,0.0,0.0);
		Y_Axis[LU_ARM].fromEuler(0.0,pitch,0.0);
		Z_Axis[LU_ARM].fromEuler(0.0,0.0,heading);

		break;
	}
	case TORSO:
	{
		TORSO_FLAG = true;
		//if ( heading > PI )
		//	heading -= 2 * PI;
		vec2.setXYZ(-roll,pitch,heading);//(-roll, pitch, heading);
		//vec2.setXYZ(-heading,pitch,roll);
		orientationQuatLAG[TORSO] = orientationQuat[TORSO];

		// These are the localy defined axis on the boards
		X_Axis[TORSO].fromEuler(-roll,0.0,0.0);
		Y_Axis[TORSO].fromEuler(0.0,pitch,0.0);
		Z_Axis[TORSO].fromEuler(0.0,0.0,heading);

		CompassVect[TORSO] = vec2;

		break;
	}
	default:
		vec2.setXYZ(-roll,pitch,heading);
		//vec2.setXYZ(-heading, pitch, roll);
		X_Axis[myID].fromEuler(-roll,0.0,0.0);
		Y_Axis[myID].fromEuler(0.0,pitch,0.0);
		Z_Axis[myID].fromEuler(0.0,0.0,heading);

	}

	return vec2;
}

// Average the gyro readings from the sensors and print the offset from the reference voltage
//bool MotionCapture::printGyroOffsets(const unsigned char &myID, const unsigned char &x, const unsigned char &y, const unsigned char &z, const unsigned char &msb)
bool MotionCapture::printGyroOffsets(const unsigned char &myID, const unsigned char &x, const unsigned char &y, const unsigned char &z, const unsigned char &msb)
{
        int i, j, val;

        // Get offset
        gyroX[myID] += (GYRO_VREF - ( ((msb & 0x30) << 4) | x ));
        gyroY[myID] += (GYRO_VREF - ( ((msb & 0x0C) << 6) | y ));
        gyroZ[myID] += (GYRO_VREF - ( ((msb & 0x03) << 8) | z ));
        counter[myID]++;

        // Be finished when every sensor has collected X amount of packets.
        // Also check for Zero because not all sensors will be operating
        bool done = true;

        for ( i = 0; i < NUM_SENSORS; i++ )
        {
                if ( !(counter[i] == 0 || counter[i] >= 2000) )
                        done = false;
        }

        // If done, print results
        if ( done )
        {
                fprintf(stdout, "\nSensor\t   X\t   Y\t   Z\n");

                // Print for user to see
                for ( i = 0; i < NUM_SENSORS; i++ )
                {
                        if ( counter[i] == 0 )
                        {
                                fprintf(stdout, "%d\t??????\t??????\t??????\n", i);
                        }
                        else
                        {
                                fprintf(stdout, "%d\t", i);
                                if ( gyroX[i] < 0 )     fprintf(stdout, "-");   else    fprintf(stdout, "+");
                                fprintf(stdout, "0x%03x\t", (abs(gyroX[i]) / counter[i]) );
                                if ( gyroY[i] < 0 )     fprintf(stdout, "-");   else    fprintf(stdout, "+");
                                fprintf(stdout, "0x%03x\t", (abs(gyroY[i]) / counter[i]) );
                                if ( gyroZ[i] < 0 )     fprintf(stdout, "-");   else    fprintf(stdout, "+");
                                fprintf(stdout, "0x%03x\t\n", (abs(gyroZ[i]) / counter[i]) );
                        }
                }

                FILE *confFile = fopen("mocap.conf", "w");

                // Make sure I can create a configuration file
                if ( confFile == NULL )
                {
                        perror("Error writing file");
                        return true;
                }
                else
                {
                        fprintf(stdout, "\nConfiguration file sucessfully created\n\n");
                }

                // Print to a configuration file
                for ( j = 0; j < 3; j++ ) // X, Y, Z
                {
                        for ( i = 0; i < NUM_SENSORS; i++ )
                        {
                                switch (j)
                                {
                                        case 0: val = gyroX[i]; break;
                                        case 1: val = gyroY[i]; break;
                                        case 2: val = gyroZ[i]; break;
                                }

                                if ( counter[i] == 0 )
                                        fprintf(confFile, "0 ");
                                else
                                        fprintf(confFile, "%d ", val / counter[i]);
                        }
                        fprintf(confFile, "\n");
                }

                fclose(confFile);
        }

        return done;
}

// Reconciling gyroscope orientation with "new" gravity orientation proivided by
// the accelerometer.
QVector MotionCapture::reconcile(QVector orientationVector, QVector stillVector)
{
	if( stillVector.isEmpty() )
		return orientationVector;
	else
	{
		return stillVector;
		// How do we do this?  May need to run tests first, and then we can decide
		// how to best use all of the data.
	}
}


// Reconciling gyroscope orientation with "new" gravity orientation proivided by
// the accelerometer, AND with the orientation provided by the compass.
QVector MotionCapture::reconcile(QVector orientationVector, QVector stillVector, QVector compassVector)
{
	if( stillVector.isEmpty() )
	{
		return compassVector;
		// Trudge on with only the gyro and compass orientations
	}
	else
	{
		return compassVector;
		// Use all three vectors to come to some consensus (some "correct" orientation)
	}
}

float MotionCapture::hex_to_dec(char msb, char lsb)
{
	float offset = 0.0;
	int16_t myCompleteInt = (msb<<8)|lsb;
	
	return(float(myCompleteInt));
//	if(myCompleteInt & 0x8000)

 
	// Since the data is in two's complement
	// we check to see if the data coming in is indeed
	// negative
//	if(msb & 0x80)
//	{
//		//offset = -65536.0;
//		offset = DBL_MIN;
//	}

//	return ( ( (msb << 8) | (unsigned char)lsb ) + offset ) * 1.0;
//	return offset - ( (msb << 8) | (unsigned char)lsb );
}

void MotionCapture::applyEKF(const unsigned char &myID, Quaternion& compass, QVector &gyro, const QVector &accel, bool has_compass, const int &g)
{
	extended_kalman[myID].update(compass, gyro, accel, has_compass, g);
}

void MotionCapture::applyEKF(const unsigned char &myID, QVector &gyro, const QVector &accel, bool has_compass, const int &g)
{
	extended_kalman[myID].update(orientationQuat[myID], gyro, accel, has_compass, g);
}

void MotionCapture::filterandRotate(const unsigned char &myID, const float &heading, const float &pitch, const float &roll, const bool &is_level, const bool &filter)
{
//	static struct timeval m_ts, m_tf;

	float heading_level = heading, heading_upright = heading;
	float pitch_level = pitch, pitch_upright = pitch;
	float roll_level = roll, roll_upright = roll;

	if ( is_level )
	{
		if ( myID == TORSO )
		{
			// filter input values as level
			if ( filter )
				kalman_level->filter(myID, heading_level, pitch_level, roll_level);

			// use as output data
			correct_heading = heading_level;
			correct_pitch = pitch_level;
			correct_roll = roll_level;

			if ( filter )
			{
				// rotate raw input data into upright orientation
				rotEuler90(heading_upright, pitch_upright, roll_upright, ROT_UP); // Rotate up 90

				// filter upright data
				kalman_upright->filter(myID, heading_upright, pitch_upright, roll_upright);
			}
		}
		else
		{
			// filter input values as level
			if ( filter )
				kalman_level->filter(myID, heading_level, pitch_level, roll_level);

			// use as output data
			correct_heading = heading_level;
			correct_pitch = pitch_level;
			correct_roll = roll_level;
		}
	}	
	else
	{
		if ( myID == TORSO ) // if TORSO
		{
			// filter input values as upright
			if ( filter )
				kalman_upright->filter(myID, heading_upright, pitch_upright, roll_upright);

			// save input values
			heading_level = heading_upright;
			pitch_level = pitch_upright;
			roll_level = roll_upright;

			// rotate to level orientation
//			rotEuler90(heading_level, pitch_level, roll_level, ROT_DOWN);

			// use as output data
			correct_heading = heading_level;
			correct_pitch = pitch_level;
			correct_roll = roll_level;

			// send level values to level filter
			if ( filter )
				kalman_level->filter(myID, heading_level, pitch_level, roll_level);
		}
		else
		{
			// rotate to level orientation
//			rotEuler90(heading_level, pitch_level, roll_level, ROT_DOWN);

			// send level values to level filter
			if ( filter )
				kalman_level->filter(myID, heading_level, pitch_level, roll_level);

			// use as output data
			correct_heading = heading_level;
			correct_pitch = pitch_level;
			correct_roll = roll_level;
		}
	}

	// store HPR which was not rotated (same angles as were passed in + filtered)
	loc_heading = (is_level)? heading_level : heading_upright;
	loc_pitch = (is_level)? pitch_level : pitch_upright;
	loc_roll = (is_level)? roll_level : roll_upright;
}

/* This function is used once applyKalman has been called.  This function will output HPR in the same orientation that it was given,
   and then HPR in the level orientation.  These HPRs will equal when the board is level, but the output will be rotated in upright front orientation
*/
void MotionCapture::extractAngles(float &level_H, float &level_P, float &level_R, float &output_H, float &output_P, float &output_R )
{
	level_H = loc_heading;
	level_P = loc_pitch;
	level_R = loc_roll;

	output_H = correct_heading;
	output_P = correct_pitch;
	output_R = correct_roll;
}

void MotionCapture::rotEuler90(float &yaw, float &pitch, float &roll, const int &dir)
{
	float	R_11, R_12, R_13,
		R_21, /*R_22, R_23,*/
		R_31, R_32, R_33;

	yaw *= PI / 180.0;
	pitch *= PI / 180.0;
	roll *= PI / 180.0;

	float cy = cos(yaw);
	float cp = cos(pitch);
	float cr = cos(roll);
	float sy = sin(yaw);
	float sp = sin(pitch);
	float sr = sin(roll);
	
	// Build rotation matrix from values
	switch (dir)
	{
		case ROT_UP:
			R_11 = -sr*sy - cr*cy*sp;
			R_12 = cr*sy - cy*sp*sr;
			R_13 = -cp*cy;
			R_21 = cr*sp*sy - cy*sr;
			//R_22 = cr*cy + sp*sr*sy;
			//R_23 = cp*sy;
			R_31 = cp*cr;
			R_32 = cp*sr;
			R_33 = -sp;
			break;
		case ROT_DOWN:
			R_11 = sr*sy + cr*cy*sp;
			R_12 = cr*sy - cy*sp*sr;
			R_13 = cp*cy;
			R_21 = cy*sr - cr*sp*sy;
			//R_22 = cr*cy + sp*sr*sy;
			//R_23 = -cp*sy;
			R_31 = -cp*cr;
			R_32 = cp*sr;
			R_33 = sp;
			break;
		default:
			yaw = 0;
			pitch = 0;
			roll = 0;
			return;
	}
	
	// Compute euler angles from rotation matrix
	if (fabs(R_31) != 1.0)
	{
		// Not a singularity point
		pitch = asin(R_31);

		if ( cos(pitch) > 0 )
		{
			roll = atan2(R_32, R_33);
			yaw = atan2(-R_21, R_11);
		}
		else
		{
			roll = atan2(-R_32, -R_33);
			yaw = atan2(R_21, -R_11);
		}
	}
	else
	{
		// Yikes! Pitch = +/-90 deg
		roll = 0; // could be anything...
		if (R_31 == -1)
		{
			// pitch = -90 deg
			pitch = -(PI/2.0);
			yaw = roll + atan2(R_12, R_13);
		}
		else
		{
			// pitch = +90 deg
			pitch = (PI/2.0);
			yaw = -roll - atan2(-R_12, -R_13);
		}
	}

	yaw *= 180.0 / PI;
	pitch *= 180.0 / PI;
	roll *= 180.0 / PI;

	// Convert -180 to 180 into 0 to 360
	if ( yaw < 0 )
		yaw += 360.0;
}

bool MotionCapture::validID(unsigned char myID)
{
	if((myID > NUM_SENSORS) || (myID < 0) || (myID ==2))
		return false;
	else
		return true;
		
}


void MotionCapture::setQuat(unsigned char myID, Quaternion myT)
{
	// check myID to be valid
	if(validID(myID))
	{
		orientationQuat[myID] = myT;
	}
}

Quaternion* MotionCapture::getQuat(unsigned char myID)
{
	if(validID(myID))
	{
		return &orientationQuat[myID];
	}
	else
		return NULL;
}

void MotionCapture::printResults(const unsigned char &myID, const QVector &accel)
{
	unsigned char w[2] = {0}, x[2] = {0}, y[2] = {0}, z[2] = {0};
	int i = 0;
	unsigned char * memblock = new unsigned char[17];
	unsigned char ax[2] = {0}, ay[2] = {0}, az[2] = {0};
	if(validID(myID))
	{
	
	// Going to alter a 'copy' of the orientationQuat in the Rotate_To_Reference function
	// which takes the locally defined orientationQuat for each sensor
	// and then rotates to the 'body' model
	
	
	OutputOrientationQuat[myID].normalize();
	floatToChar(OutputOrientationQuat[myID].w, w);
	floatToChar(OutputOrientationQuat[myID].toVector().x, x);
	floatToChar(OutputOrientationQuat[myID].toVector().y, y);
	floatToChar(OutputOrientationQuat[myID].toVector().z, z);

	floatToChar(accel.x, ax);
	floatToChar(accel.y, ay);
	floatToChar(accel.z, az);

	// Problem with printing chars:  OPOST flag needs to be disabled on /dev/ttyS2, 
	// otherwise special characters like 0xA (line feed) will turn into 0x0D 0x0A
	// ID, W, X, Y, Z

//	if ( !readable )
//		fprintf(stdout, "%c%c%c%c%c%c%c%c%c%c%c", 0xFE, myID, w[0], w[1], x[0], x[1], y[0], y[1], z[0], z[1], 0xFF);
//	else
	
//fprintf(myOutputStream,"%02x%x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",0xFE, myID, w[0],w[1], x[0], x[1], y[0], y[1], z[0], z[1], ax[0], ax[1], ay[0], ay[1], az[0], az[1], 0xFF);
//fprintf(stdout,"%02x%x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",0xFE, myID, w[0],w[1], x[0], x[1], y[0], y[1], z[0], z[1], ax[0], ax[1], ay[0], ay[1], az[0], az[1], 0xFF);
	//fprintf(stdout,"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",0xFE,myID, w[0],w[1], x[0], x[1], y[0], y[1], z[0], z[1], ax[0], ax[1], ay[0], ay[1], az[0], az[1], 0xFF);
	
	
	memblock[0] = 0xFE;
	memblock[1] = myID;
	memblock[2] = w[0];
	memblock[3] = w[1];
	memblock[4] = x[0];
	memblock[5] = x[1];
	memblock[6] = y[0];
	memblock[7] = y[1];
	memblock[8] = z[0];
	memblock[9] = z[1];
	memblock[10] = ax[0];
	memblock[11] = ax[1];
	memblock[12] = ay[0];
	memblock[13] = ay[1];
	memblock[14] = az[0];
	memblock[15] = az[1];
	memblock[16] = 0xFF;

	myOutputStream.write(reinterpret_cast<char*>(memblock),17);		
	delete[] memblock;
	//fflush(stdout);
	}
}


void MotionCapture::charToFloat(unsigned char in[2], float &out)
{
	short ctos;
	ctos = (in[1]<<8)+in[0];
	out = ctos /10000.0;
}

void MotionCapture::floatToChar(const float &in, unsigned char out[2])
{
	short ftoi = (short)(in * 10000.0);
	unsigned char *floatChar = (unsigned char*)&ftoi;

	out[0] = floatChar[0];
	out[1] = floatChar[1];

}

/*
Rotate the system to the TORSO reference setup.
*/

void MotionCapture::Rotate_To_TorsoReference(unsigned char myID)
{
QVector TorsoTemp(0,0,0);
QVector RightArmTemp(1,1,0);
QVector LeftArmTemp(1,1,0);
QVector RightLegTemp(0,1,0);
QVector LeftLegTemp(0,1,0);
QVector Left_UpperArm(0,0,0);
Quaternion RightArmTempQuat(0,RightArmTemp);
Quaternion LeftArmTempQuat(0,LeftArmTemp);
Quaternion RightLegTempQuat(0,RightLegTemp);
Quaternion LeftLegTempQuat(0,LeftLegTemp);
Quaternion Correction_TEMP;
Quaternion TEMPQUAT;
Quaternion TEMPQUAT2;
Quaternion TEMPQUAT3;
QVector TEMPVECT;
QVector TEMPVECT2;
QVector TEMPVECT3;
if(validID(myID))
{
if(TORSO_FLAG)
{
switch ( myID )
{
	orientationQuat[TORSO].normalize();
	case RU_ARM:{
		if(SensorSet[TORSO])
		{
//		fprintf(stdout,"RU_ARM\n");

		orientationQuat[RU_ARM].normalize();
		orientationQuat[TORSO].normalize();
		TORSO_Z_RIGHTSIDE.normalize();
		// This should take out the heading value of the
		// TORSO from the Heading of the Upper Arm
		// With this done the system then is defined starting at
		// Zero rotation on the body overall, the difference in 
		// the arm 'should' be +/- 90 offset from the TORSO with
		// arms at the side.
		OutputOrientationQuat[RU_ARM] = TORSO_Z_RIGHTSIDE.conjugate() * orientationQuat[RU_ARM];
		//OutputOrientationQuat[RU_ARM] = orientationQuat[RU_ARM] * TORSO_Z_RIGHTSIDE.conjugate();
		OutputOrientationQuat[TORSO].normalize();

		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[RU_ARM] = TEMPQUAT.rotate(OutputOrientationQuat[RU_ARM],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[RU_ARM].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[RU_ARM].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[RU_ARM].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[RU_ARM].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[RU_ARM].vector.z = TEMPVECT2.z;
		OutputOrientationQuat[RU_ARM].normalize();
		OutputOrientationQuat[RU_ARM].print(RU_ARM);


		SensorSet[RU_ARM] = true;
//		fprintf(stdout,"RU_ARM\n");

		}
		break;
	}
	case RL_ARM:{
		if(SensorSet[RU_ARM])
		{
//		fprintf(stdout,"RL_ARM\n");

		// BEGIN REMOVE JHD 5-28
		// Make sure we opporate on normalized quats
		OutputOrientationQuat[RU_ARM].normalize();
		orientationQuat[RL_ARM].normalize();
		// Bring the RL_ARM into the same relative position
		OutputOrientationQuat[RL_ARM] = TORSO_Z_RIGHTSIDE.conjugate() * orientationQuat[RL_ARM];
		// Now find the difference to the upper segment
		OutputOrientationQuat[RL_ARM].normalize();
		OutputOrientationQuat[RL_ARM] = OutputOrientationQuat[RU_ARM].conjugate() * OutputOrientationQuat[RL_ARM];
		OutputOrientationQuat[RL_ARM].normalize();
		
		// END REMOVE

		// Find difference between orignal RU_ARM and RL_ARM quats
		//OutputOrientationQuat[RL_ARM] = orientationQuat[RU_ARM].conjugate() * orientationQuat[RL_ARM];
		//OutputOrientationQuat[RL_ARM].normalize();

		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[RL_ARM] = TEMPQUAT.rotate(OutputOrientationQuat[RL_ARM],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[RL_ARM].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[RL_ARM].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[RL_ARM].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[RL_ARM].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[RL_ARM].vector.z = TEMPVECT2.z;
		OutputOrientationQuat[RL_ARM].normalize();
		SensorSet[RL_ARM] = true;
		OutputOrientationQuat[RL_ARM].print(RL_ARM);
//		fprintf(stdout,"RL_ARM\n");

		}
		break;
	}
	case RU_LEG:{
		if(SensorSet[TORSO])
		{
//		fprintf(stdout,"RU_LEG\n");

		orientationQuat[RU_LEG].normalize();
		orientationQuat[TORSO].normalize();
		TORSO_Z_RIGHTSIDE.normalize();
		OutputOrientationQuat[RU_LEG] = TORSO_Z_RIGHTSIDE.conjugate() * orientationQuat[RU_LEG];
		//OutputOrientationQuat[RU_LEG] = orientationQuat[RU_LEG] * TORSO_Z_RIGHTSIDE.conjugate();

		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[RU_LEG] = TEMPQUAT.rotate(OutputOrientationQuat[RU_LEG],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[RU_LEG].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[RU_LEG].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[RU_LEG].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[RU_LEG].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[RU_LEG].vector.z = TEMPVECT2.z;
		OutputOrientationQuat[RU_LEG].normalize();
		
		OutputOrientationQuat[RU_LEG].print(RU_LEG);
		
		SensorSet[RU_LEG] = true;
//		fprintf(stdout,"RU_LEG\n");

		}
		break;
	}
	case RL_LEG:{
		if(SensorSet[RU_LEG])
		{
//		fprintf(stdout,"RL_LEG\n");

		// REMOVING JHD 5-28
		// Make sure we opporate on normalized quats
		OutputOrientationQuat[RU_LEG].normalize();
		orientationQuat[RL_LEG].normalize();
		// Bring the RL_LEG into the same relative position
		OutputOrientationQuat[RL_LEG] = TORSO_Z_RIGHTSIDE.conjugate() * orientationQuat[RL_LEG];
		// Now find the difference to the upper segment

		OutputOrientationQuat[RL_LEG].normalize();
		OutputOrientationQuat[RL_LEG] = OutputOrientationQuat[RU_LEG].conjugate() * OutputOrientationQuat[RL_LEG];
		OutputOrientationQuat[RL_LEG].normalize();
		// END REMOVE

		// Find difference between orignal RU_LEG and RL_LEG quats
		//OutputOrientationQuat[RL_LEG] = orientationQuat[RU_LEG].conjugate() * orientationQuat[RL_LEG];
		//OutputOrientationQuat[RL_LEG].normalize();



		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[RL_LEG] = TEMPQUAT.rotate(OutputOrientationQuat[RL_LEG],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[RL_LEG].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[RL_LEG].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[RL_LEG].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[RL_LEG].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[RL_LEG].vector.z = TEMPVECT2.z;
		OutputOrientationQuat[RL_LEG].normalize();
		SensorSet[RL_LEG] = true;
		OutputOrientationQuat[RL_LEG].print(RL_LEG);
//		fprintf(stdout,"RL_LEG\n");

		}
		break;
	}
	case LU_ARM:{
		if(SensorSet[TORSO])
		{
//		fprintf(stdout,"LU_ARM\n");

		orientationQuat[LU_ARM].normalize();
		orientationQuat[TORSO].normalize();
		TORSO_Z_LEFTSIDE.normalize();
		OutputOrientationQuat[LU_ARM] = TORSO_Z_LEFTSIDE.conjugate() * orientationQuat[LU_ARM];
		//OutputOrientationQuat[LU_ARM] = orientationQuat[LU_ARM] * TORSO_Z_LEFTSIDE.conjugate();

		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[LU_ARM] = TEMPQUAT.rotate(OutputOrientationQuat[LU_ARM],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[LU_ARM].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[LU_ARM].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[LU_ARM].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[LU_ARM].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[LU_ARM].vector.z = -TEMPVECT2.z;
		OutputOrientationQuat[LU_ARM].normalize();
		OutputOrientationQuat[LU_ARM].print(LU_ARM);
//		fprintf(stdout,"LU_ARM\n");

		SensorSet[LU_ARM] = true;
		}
	break;
	}
	case LL_ARM:{
		if(SensorSet[LU_ARM])
		{
//		fprintf(stdout,"LL_ARM\n");

		// Removing the following
		Correction_TEMP = OutputOrientationQuat[LU_ARM];//.normalize();
		Correction_TEMP.normalize();
		Correction_TEMP.vector.z = (-1.0) * Correction_TEMP.vector.z;
		Correction_TEMP.normalize();

		orientationQuat[LL_ARM].normalize();

		// Bring the LL_ARM into the same relative position
		OutputOrientationQuat[LL_ARM] = TORSO_Z_LEFTSIDE.conjugate() * orientationQuat[LL_ARM];
		// Now find the difference to the upper segment
		
		OutputOrientationQuat[LL_ARM].normalize();
		OutputOrientationQuat[LL_ARM] = Correction_TEMP.conjugate() * OutputOrientationQuat[LL_ARM];
		OutputOrientationQuat[LL_ARM].normalize();
		// END removal JHD 5-28

		// Find difference between orignal LU_ARM and LL_ARM quats
		//OutputOrientationQuat[LL_ARM] = orientationQuat[LU_ARM].conjugate() * orientationQuat[LL_ARM];
		//OutputOrientationQuat[LL_ARM].normalize();

		//OutputOrientationQuat[LL_ARM].print(LL_ARM);
		
		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[LL_ARM] = TEMPQUAT.rotate(OutputOrientationQuat[LL_ARM],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[LL_ARM].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[LL_ARM].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[LL_ARM].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[LL_ARM].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[LL_ARM].vector.z = -TEMPVECT2.z;
		OutputOrientationQuat[LL_ARM].normalize();
		OutputOrientationQuat[LL_ARM].print(LL_ARM);
		SensorSet[LL_ARM] = true;
//		fprintf(stdout,"LL_ARM\n");

		}
		break;
	}
	case LU_LEG:{
		if(SensorSet[TORSO])
		{
//		fprintf(stdout,"LU_LEG\n");

		orientationQuat[LU_LEG].normalize();
		orientationQuat[TORSO].normalize();
		TORSO_Z_LEFTSIDE.normalize();
		OutputOrientationQuat[LU_LEG] = TORSO_Z_LEFTSIDE.conjugate() * orientationQuat[LU_LEG];
		//OutputOrientationQuat[LU_LEG] = orientationQuat[LU_LEG] * TORSO_Z_LEFTSIDE.conjugate();

		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[LU_LEG] = TEMPQUAT.rotate(OutputOrientationQuat[LU_LEG],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[LU_LEG].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[LU_LEG].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[LU_LEG].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[LU_LEG].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[LU_LEG].vector.z = -TEMPVECT2.z;
		OutputOrientationQuat[LU_LEG].print(LU_LEG);


		SensorSet[LU_LEG] = true;
//		fprintf(stdout,"LU_LEG\n");

		}
		break;
	}
	case LL_LEG:{
		if(SensorSet[LU_LEG])
		{
//		fprintf(stdout,"LL_LEG\n");
		// Make sure we opporate on normalized quats
		// Make sure to not double negate teh lowe left limbs
		// REMOVING JHD 5-28
		Correction_TEMP = OutputOrientationQuat[LU_LEG];//.normalize();
		Correction_TEMP.normalize();
		Correction_TEMP.vector.z = (-1.0) * Correction_TEMP.vector.z;
		Correction_TEMP.normalize();

		orientationQuat[LL_LEG].normalize();
		// Bring the LL_LEG into the same relative position
		OutputOrientationQuat[LL_LEG] = TORSO_Z_LEFTSIDE.conjugate() * orientationQuat[LL_LEG];
		// Now find the difference to the upper segment
		OutputOrientationQuat[LL_LEG].normalize();
		OutputOrientationQuat[LL_LEG] = Correction_TEMP.conjugate() * OutputOrientationQuat[LL_LEG];
		OutputOrientationQuat[LL_LEG].normalize();
		// END REMOVE

		// Find difference between orignal LU_LEG and LL_LEG quats
		//OutputOrientationQuat[LL_LEG] = orientationQuat[LU_LEG].conjugate() * orientationQuat[LL_LEG];
		//OutputOrientationQuat[LL_LEG].normalize();

		
		// OK so this part below rotates the
		// vector <1,0,0> by the quaternion
		TEMPVECT = QVector(1,0,0);
		// IF this system is at rest JUMPSUIT, the vector should 
		// be ~<0,-1,0> where the y-Axis + is through your head.
		OutputOrientationQuat[LL_LEG] = TEMPQUAT.rotate(OutputOrientationQuat[LL_LEG],TEMPVECT);
		TEMPVECT2 = OutputOrientationQuat[LL_LEG].toVector();
		// Now then we are going to find the resultant
		// quaternion that rotates the vector <1,0,0> to the
		// above vector using the halfVNorm stuff.
		TEMPVECT3 = TEMPVECT2 + TEMPVECT;
		TEMPVECT3 = TEMPVECT3.unitVector();
		OutputOrientationQuat[LL_LEG].w = TEMPVECT.vectorDot(TEMPVECT3);	
		TEMPVECT2 = TEMPVECT.vectorCross(TEMPVECT3);
		OutputOrientationQuat[LL_LEG].vector.x = TEMPVECT2.x;
		OutputOrientationQuat[LL_LEG].vector.y = TEMPVECT2.y;
		OutputOrientationQuat[LL_LEG].vector.z = -TEMPVECT2.z;
		OutputOrientationQuat[LL_LEG].normalize();
		//fprintf(stdout,"LL_LEG\n");
		OutputOrientationQuat[LL_LEG].print(LL_LEG);
		SensorSet[LL_LEG] = true;
		//fprintf(stdout,"LL_LEG_END\n");

		}
	break;
	}
	case TORSO:{
//		fprintf(stdout,"TORSO\n");
		orientationQuat[TORSO].normalize();
		orientationQuatLAG[TORSO].normalize();
		TORSO_Z.normalize();
		// Going to take out the Z-Axis Rotation 'Heading'
		// This puts the Torso into always facing forward
		// We keep the local defined pitch/roll on the TORSO
		OutputOrientationQuat[TORSO] = TORSO_Z.conjugate() * orientationQuat[TORSO];
		// For the sake of reducing noise going to just say
		// that the torso is always <1,0,0,0> ONLY DO THIS IF
		// WE DONT WANT THE ACTUAL QUATS
		//OutputOrientationQuat[TORSO] = orientationQuat[TORSO].conjugate() * orientationQuat[TORSO];
		// RGIHT HAND RULE FOR ROTATION SIGNS
		// changed this from PI/2.0 to -PI/2.0
		TEMPQUAT.fromEuler(0.0,0.0,PI/2.0);// = Quaternion(1,TEMPVECT);
		TEMPQUAT.normalize();
		TORSO_Z_RIGHTSIDE = TORSO_Z * TEMPQUAT;
		TORSO_Z_RIGHTSIDE.normalize();

		TEMPQUAT.fromEuler(0.0,0.0,-PI/2.0);// = Quaternion(1,TEMPVECT);
		TEMPQUAT.normalize();
		TORSO_Z_LEFTSIDE = TORSO_Z * TEMPQUAT;
		TORSO_Z_LEFTSIDE.normalize();		

//		OutputOrientationQuat[TORSO].print(TORSO);
		SensorSet[TORSO] = true;
//		fprintf(stdout,"TORSO\n");
	break;
	}
}
	OutputOrientationQuat[myID].normalize();
}
}
}



/*
void MotionCapture::Rotate_To_TorsoReference(unsigned char myID)
{
QVector TorsoTemp(0,0,0);
QVector RightArmTemp(1,1,0);
QVector LeftArmTemp(1,1,0);
QVector RightLegTemp(0,1,0);
QVector LeftLegTemp(0,1,0);
QVector Left_UpperArm(0,0,0);
Quaternion RightArmTempQuat(0,RightArmTemp);
Quaternion LeftArmTempQuat(0,LeftArmTemp);
Quaternion RightLegTempQuat(0,RightLegTemp);
Quaternion LeftLegTempQuat(0,LeftLegTemp);
Quaternion Correction_TEMP;
Quaternion TEMPQUAT;
Quaternion TEMPQUAT2;
Quaternion TEMPQUAT3;
QVector TEMPVECT;
QVector TEMPVECT2;
QVector TEMPVECT3;
if(validID(myID))
{
switch ( myID )
{
	orientationQuat[TORSO].normalize();
	case RU_ARM:{
		orientationQuat[RU_ARM].normalize();
		orientationQuat[TORSO].normalize();
		OutputOrientationQuat[TORSO].normalize();
		// Make the calcs go in order TORSO -> LU_* or RU_*
		if(BaselineOffsetSet[TORSO])
		{
		if(!BaselineOffsetSet[RU_ARM])
		{
			BaselineOffsetQuaternion[RU_ARM] = orientationQuat[RU_ARM];
			TEMPVECT = orientationQuat[RU_ARM].toVector();
			BaselineOffsetPoint[RU_ARM] = TEMPVECT;
			BaselineOffsetPoint[RU_ARM] = BaselineOffsetPoint[RU_ARM]* (-1.0);
			BaselineOffsetSet[RU_ARM] = true;

			X_Axis_Baseline[RU_ARM] = X_Axis[RU_ARM]; 
			Y_Axis_Baseline[RU_ARM] = Y_Axis[RU_ARM]; 
			Z_Axis_Baseline[RU_ARM] = Z_Axis[RU_ARM]; 

		}
		// Test to make sure I have a ~[1,0,0,0] since I should 
		// Gives me baseline angles.
		// Should be ~<1,0,0,0> if first starting out
		TEMPQUAT = orientationQuat[RU_ARM] * BaselineOffsetQuaternion[RU_ARM].conjugate();
		RU_ARM_ABS = TEMPQUAT.conjugate();
		OutputOrientationQuat[RU_ARM] = TEMPQUAT;//
		OutputOrientationQuat[RU_ARM] = OutputOrientationQuat[RU_ARM] * TORSO_ABS;
		// Now Lets get the baseline single axis of rotation
		// for the LU_ARM
		TEMPQUAT3.fromEuler(0.0,-PI/2.0,0.0);
		TEMPQUAT3.normalize();
		OutputOrientationQuat[RU_ARM] = OutputOrientationQuat[RU_ARM] * TEMPQUAT3;
		}

	break;
	}
	case RL_ARM:{
		if(BaselineOffsetSet[TORSO])
		{
		if(BaselineOffsetSet[RU_ARM])
		{
		if(!BaselineOffsetSet[RL_ARM])
		{
			BaselineOffsetQuaternion[RL_ARM] = orientationQuat[RL_ARM];
			TEMPVECT = orientationQuat[RL_ARM].toVector();
			BaselineOffsetPoint[RL_ARM] = TEMPVECT;
			BaselineOffsetPoint[RL_ARM] = BaselineOffsetPoint[RL_ARM]* (-1.0);
			BaselineOffsetSet[RL_ARM] = true;
		}
		TEMPQUAT3.fromEuler(0.0,PI/2.0,0.0);
		TEMPQUAT = orientationQuat[RL_ARM] * BaselineOffsetQuaternion[RL_ARM].conjugate();
		OutputOrientationQuat[RL_ARM] = TEMPQUAT;//
		OutputOrientationQuat[RL_ARM] = OutputOrientationQuat[RL_ARM] * RU_ARM_ABS;
		OutputOrientationQuat[RL_ARM] = OutputOrientationQuat[RL_ARM] * TORSO_ABS;

		//OutputOrientationQuat[RL_ARM] = TEMPQUAT*TEMPQUAT3;//.rotate(OutputOrientationQuat[TORSO].conjugate(),TEMPVECT);
		}
	}

	break;
	}
	case RU_LEG:{
		orientationQuat[RU_ARM].normalize();
		orientationQuat[TORSO].normalize();
		OutputOrientationQuat[TORSO].normalize();
		// Make the calcs go in order TORSO -> LU_* or RU_*
		if(BaselineOffsetSet[TORSO])
		{
		if(!BaselineOffsetSet[RU_LEG])
		{
			BaselineOffsetQuaternion[RU_LEG] = orientationQuat[RU_LEG];
			TEMPVECT = orientationQuat[RU_LEG].toVector();
			BaselineOffsetPoint[RU_LEG] = TEMPVECT;
			BaselineOffsetPoint[RU_LEG] = BaselineOffsetPoint[RU_LEG]* (-1.0);
			BaselineOffsetSet[RU_LEG] = true;

			X_Axis_Baseline[RU_LEG] = X_Axis[RU_LEG]; 
			Y_Axis_Baseline[RU_LEG] = Y_Axis[RU_LEG]; 
			Z_Axis_Baseline[RU_LEG] = Z_Axis[RU_LEG]; 

		}
		// Test to make sure I have a ~[1,0,0,0] since I should 
		// be in the same frame of reference
		// TEMPQUAT2 is the orientation/rotation on the Limb by 
		// the Torso, with the Limb specific rotation taken out 
		// This prevents double rotation
		// Gives me a 'Zero'd out representation of the 
		// quaternion
		TEMPQUAT3.fromEuler(0.0,0.0,-PI/2.0);
		TEMPQUAT3.normalize();

		// Gives me baseline angles.
		// Should be ~<1,0,0,0> if first starting out
		TEMPQUAT = orientationQuat[RU_LEG] * BaselineOffsetQuaternion[RU_LEG].conjugate();
		OutputOrientationQuat[RU_LEG] = TEMPQUAT;//
		RU_LEG_ABS = TEMPQUAT.conjugate();

		TEMPQUAT = X_Axis[TORSO].conjugate();
		TEMPQUAT.normalize();
		OutputOrientationQuat[RU_LEG] = OutputOrientationQuat[RU_LEG] * TORSO_ABS;
		// Now Lets get the baseline single axis of rotation
		// for the LU_ARM
		TEMPQUAT3.fromEuler(0.0,-PI/2.0,0.0);
		TEMPQUAT3.normalize();
		OutputOrientationQuat[RU_LEG] = OutputOrientationQuat[RU_LEG] * TEMPQUAT3;
		}
	break;
	}
	case RL_LEG:{
		if(BaselineOffsetSet[TORSO])
		{
		if(BaselineOffsetSet[RU_LEG])
		{
		if(!BaselineOffsetSet[RL_LEG])
		{
			BaselineOffsetQuaternion[RL_LEG] = orientationQuat[RL_LEG];
			TEMPVECT = orientationQuat[RL_LEG].toVector();
			BaselineOffsetPoint[RL_LEG] = TEMPVECT;
			BaselineOffsetPoint[RL_LEG] = BaselineOffsetPoint[RL_LEG]* (-1.0);
			BaselineOffsetSet[RL_LEG] = true;
		}
		TEMPQUAT3.fromEuler(0.0,PI/2.0,0.0);
		TEMPQUAT = orientationQuat[RL_LEG] * BaselineOffsetQuaternion[RL_LEG].conjugate();
		OutputOrientationQuat[RL_LEG] = TEMPQUAT;//
		OutputOrientationQuat[RL_LEG] = OutputOrientationQuat[RL_LEG] * RU_LEG_ABS;
		OutputOrientationQuat[RL_LEG] = OutputOrientationQuat[RL_LEG] * TORSO_ABS;

		//OutputOrientationQuat[RL_LEG] = TEMPQUAT*TEMPQUAT3;//.rotate(OutputOrientationQuat[TORSO].conjugate(),TEMPVECT);
		}
		}

	break;
	}
	case LU_ARM:{
		// The setup is as follows:
		// using strictly Quaternion Math
		// We take the computed quaternion from the compass 
		// update this represents enough information to place 
		// the limb into space at a point completely.
		// Ok well that sucks if we want to pull this into 
		// reference with the TORSO by multiplying by it's 
		// quaternion.
		// So lets subtract the quaternions to get the necessary 
		// rotation for the limb WRT the TORSO ORDER WILL MATTER 
		// HERE!!!
		// To do so we need to multiply by the TORSO conjugate
		// which will effectively subtract it's heading 
		// information out

		orientationQuat[LU_ARM].normalize();
		orientationQuat[TORSO].normalize();
		OutputOrientationQuat[TORSO].normalize();
		// Make the calcs go in order TORSO -> LU_* or RU_*
		if(BaselineOffsetSet[TORSO])
		{
		if(!BaselineOffsetSet[LU_ARM])
		{
			BaselineOffsetQuaternion[LU_ARM] = orientationQuat[LU_ARM];
			TEMPVECT = orientationQuat[LU_ARM].toVector();
			BaselineOffsetPoint[LU_ARM] = TEMPVECT;
			BaselineOffsetPoint[LU_ARM] = BaselineOffsetPoint[LU_ARM]* (-1.0);
			BaselineOffsetSet[LU_ARM] = true;

			X_Axis_Baseline[LU_ARM] = X_Axis[LU_ARM]; 
			Y_Axis_Baseline[LU_ARM] = Y_Axis[LU_ARM]; 
			Z_Axis_Baseline[LU_ARM] = Z_Axis[LU_ARM]; 

		}
		// Test to make sure I have a ~[1,0,0,0] since I should 
		// be in the same frame of reference
		// TEMPQUAT2 is the orientation/rotation on the Limb by 
		// the Torso, with the Limb specific rotation taken out 
		// This prevents double rotation
		// Gives me a 'Zero'd out representation of the 
		// quaternion
		//TEMPQUAT3.fromEuler(0.0,0.0,-PI/2.0);
		//TEMPQUAT3.normalize();

		// Gives me baseline angles.
		// Should be ~<1,0,0,0> if first starting out
		TEMPQUAT = orientationQuat[LU_ARM] * BaselineOffsetQuaternion[LU_ARM].conjugate();
		OutputOrientationQuat[LU_ARM] = TEMPQUAT;//
		OutputOrientationQuat[LU_ARM] = OutputOrientationQuat[LU_ARM] * TORSO_ABS;
		LU_ARM_ABS = OutputOrientationQuat[LU_ARM].conjugate();
		// This just lets the jumpsuit position start as arms down at side, and
		// and similarly legs straight down from torso.
		TEMPQUAT3.fromEuler(0.0,-PI/2.0,0.0);
		TEMPQUAT3.normalize();
		OutputOrientationQuat[LU_ARM] = OutputOrientationQuat[LU_ARM] * TEMPQUAT3;// * OutputOrientationQuat[TORSO].conjugate();
		//OutputOrientationQuat[LU_ARM] = OutputOrientationQuat[LU_ARM].conjugate();
		}

	break;
	}
	case LL_ARM:{
		if(BaselineOffsetSet[TORSO])
		{
		if(BaselineOffsetSet[LU_ARM])
		{
		if(!BaselineOffsetSet[LL_ARM])
		{
			BaselineOffsetQuaternion[LL_ARM] = orientationQuat[LL_ARM];
			TEMPVECT = orientationQuat[LL_ARM].toVector();
			BaselineOffsetPoint[LL_ARM] = TEMPVECT;
			BaselineOffsetPoint[LL_ARM] = BaselineOffsetPoint[LL_ARM]* (-1.0);
			BaselineOffsetSet[LL_ARM] = true;
		}
		TEMPQUAT3.fromEuler(0.0,PI/2.0,0.0);
		// This should give ~<1,0,0,0>
		TEMPQUAT = orientationQuat[LL_ARM] * BaselineOffsetQuaternion[LL_ARM].conjugate();
		OutputOrientationQuat[LL_ARM] = TEMPQUAT;//
		OutputOrientationQuat[LL_ARM] = OutputOrientationQuat[LL_ARM] * LU_ARM_ABS;
		OutputOrientationQuat[LL_ARM] = OutputOrientationQuat[LL_ARM] * TORSO_ABS;

		//OutputOrientationQuat[LL_ARM] = TEMPQUAT*TEMPQUAT3;//.rotate(OutputOrientationQuat[TORSO].conjugate(),TEMPVECT);
		}
	}
	break;
	}
	case LU_LEG:{
		orientationQuat[LU_LEG].normalize();
		orientationQuat[TORSO].normalize();
		OutputOrientationQuat[TORSO].normalize();
		// Make the calcs go in order TORSO -> LU_* or RU_*
		if(BaselineOffsetSet[TORSO])
		{
		if(!BaselineOffsetSet[LU_LEG])
		{
			BaselineOffsetQuaternion[LU_LEG] = orientationQuat[LU_LEG];
			TEMPVECT = orientationQuat[LU_LEG].toVector();
			BaselineOffsetPoint[LU_LEG] = TEMPVECT;
			BaselineOffsetPoint[LU_LEG] = BaselineOffsetPoint[LU_LEG]* (-1.0);
			BaselineOffsetSet[LU_LEG] = true;

			X_Axis_Baseline[LU_LEG] = X_Axis[LU_LEG]; 
			Y_Axis_Baseline[LU_LEG] = Y_Axis[LU_LEG]; 
			Z_Axis_Baseline[LU_LEG] = Z_Axis[LU_LEG]; 

		}
		// Test to make sure I have a ~[1,0,0,0] since I should 
		// Gives me baseline angles.
		// Should be ~<1,0,0,0> if first starting out
		TEMPQUAT = orientationQuat[LU_LEG] * BaselineOffsetQuaternion[LU_LEG].conjugate();
		LU_LEG_ABS = TEMPQUAT.conjugate();

		OutputOrientationQuat[LU_LEG] = TEMPQUAT;//
		OutputOrientationQuat[LU_LEG] = OutputOrientationQuat[LU_LEG] * TORSO_ABS;
		TEMPQUAT3.fromEuler(0.0,-PI/2.0,0.0);
		TEMPQUAT3.normalize();
		OutputOrientationQuat[LU_LEG] = OutputOrientationQuat[LU_LEG] * TEMPQUAT3;// * OutputOrientationQuat[TORSO].conjugate();
		}
	break;
	}
	case LL_LEG:{
		if(BaselineOffsetSet[TORSO])
		{
		if(BaselineOffsetSet[LU_LEG])
		{
		if(!BaselineOffsetSet[LL_LEG])
		{
			BaselineOffsetQuaternion[LL_LEG] = orientationQuat[LL_LEG];
			TEMPVECT = orientationQuat[LL_LEG].toVector();
			BaselineOffsetPoint[LL_LEG] = TEMPVECT;
			BaselineOffsetPoint[LL_LEG] = BaselineOffsetPoint[LU_LEG]* (-1.0);
			BaselineOffsetSet[LL_LEG] = true;
		}
		
		// The following changes will break compatibility to the
		// visualization program
		// For the lower segments:
		// The output will be the Baseline difference of the 
		// local quaternion. This gives the absolute angle 
		// encoded in quaternion space.
		
		// Next the baselined quaternion will be combined with 
		// the conjugate of the Upper_Segment
		// This will give the relative additional angles 
		// necessary w.r.t the upper segment. Finally
		// The output will be brought into reference with the 
		// TORSO by multiplying through with the Torsoconjugate 
		// TORSOABS.

		TEMPQUAT3.fromEuler(0.0,PI/2.0,0.0);
		TEMPQUAT = orientationQuat[LL_LEG] * BaselineOffsetQuaternion[LL_LEG].conjugate();
		OutputOrientationQuat[LL_LEG] = TEMPQUAT;//
		OutputOrientationQuat[LL_LEG] = OutputOrientationQuat[LL_LEG] * LU_LEG_ABS;
		OutputOrientationQuat[LL_LEG] = OutputOrientationQuat[LL_LEG] * TORSO_ABS;
		//OutputOrientationQuat[LL_LEG] = TEMPQUAT*TEMPQUAT3;//.rotate(OutputOrientationQuat[TORSO].conjugate(),TEMPVECT);

		}
		}
	break;
	}
	case TORSO:{
		orientationQuat[TORSO].normalize();
		orientationQuatLAG[TORSO].normalize();
		
		if(!BaselineOffsetSet[TORSO] && WaitCounter >5)
		{
			// LEFTSIDE Correction
			TEMPQUAT3.fromEuler(0.0,0.0,-PI/2);		
			TEMPQUAT3.normalize();

			BaselineOffsetQuaternion[TORSO] = orientationQuat[TORSO];
			TEMPVECT = orientationQuat[TORSO].toVector();
			BaselineOffsetPoint[TORSO] = TEMPVECT;
			BaselineOffsetPoint[TORSO] = BaselineOffsetPoint[TORSO]* (-1.0);
			BaselineOffsetSet[TORSO] = true;
			BaselineOffsetQuaternionLS[LU_ARM] = BaselineOffsetQuaternion[TORSO] * TEMPQUAT3;
			TEMPQUAT3.fromEuler(0.0,0.0,PI/2);		
			TEMPQUAT3.normalize();
			BaselineOffsetQuaternionRS[RU_ARM] = BaselineOffsetQuaternion[TORSO] * TEMPQUAT3;
			WaitCounter = 10;
			
			X_Axis_Baseline[TORSO] = X_Axis[TORSO]; 
			Y_Axis_Baseline[TORSO] = Y_Axis[TORSO]; 
			Z_Axis_Baseline[TORSO] = Z_Axis[TORSO]; 
		}
		orientationQuat[TORSO].normalize();
		WaitCounter ++;
		OutputOrientationQuat[TORSO] = orientationQuat[TORSO] * BaselineOffsetQuaternion[TORSO].conjugate();
		TORSO_ABS = OutputOrientationQuat[TORSO].conjugate();
		OutputOrientationQuat[TORSO] = OutputOrientationQuat[TORSO] * TORSO_ABS;
		/*TEMPQUAT.fromEuler(0.0,PI/2.0,0.0);//Quaternion(1,TEMPVECT);
		TEMPQUAT.normalize();
		OutputOrientationQuat[TORSO] = OutputOrientationQuat[TORSO] * TEMPQUAT;
		TEMPQUAT.fromEuler(0.0,0.0,-PI/2.0);//Quaternion(1,TEMPVECT);
		TEMPQUAT.normalize();
		OutputOrientationQuat[TORSO] = OutputOrientationQuat[TORSO] * TEMPQUAT;
		*/
		//TEMPVECT = OutputOrientationQuat[TORSO].toVector();
		//TEMPVECT = TEMPVECT * 5.0;
		//fprintf(stdout,"TORSO x[%1.3f]y[%1.3f]z[%1.3f]\n",TEMPVECT.x,TEMPVECT.y,TEMPVECT.z);

	break;
	}
}
	//OutputOrientationQuat[myID] = orientationQuat[myID];
	OutputOrientationQuat[myID].normalize();
	ReferenceOrientationQuat[myID].normalize();
}
}
*/
QuaternionArray MotionCapture::fillAndGetQuatArray() {
	QuaternionArray QuatArray;
	QuatArray.quats[0] = OutputOrientationQuat[TORSO];
	QuatArray.quats[1] = OutputOrientationQuat[LU_ARM];
	QuatArray.quats[2] = OutputOrientationQuat[LL_ARM];
	QuatArray.quats[3] = OutputOrientationQuat[RU_ARM];
	QuatArray.quats[4] = OutputOrientationQuat[RL_ARM];
	QuatArray.quats[5] = OutputOrientationQuat[LU_LEG];
	QuatArray.quats[6] = OutputOrientationQuat[LL_LEG];
	QuatArray.quats[7] = OutputOrientationQuat[RU_LEG];
	QuatArray.quats[8] = OutputOrientationQuat[RL_LEG];
	return QuatArray;
}

