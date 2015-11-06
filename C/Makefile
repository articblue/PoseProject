all: motioncapture

motioncapture: main.cpp MotionCapture.h MotionCapture.cpp 
	g++ -Wall main.cpp QVector.cpp Matrix.cpp Quaternion.cpp MotionCapture.cpp Kalman.cpp SuperKalman.cpp AngleContainer.cpp -o motioncapture
clean:
	rm motioncapture
