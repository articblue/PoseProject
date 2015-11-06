#!/bin/bash
# Compile my C Code

#Clean Everything
rm -rf  ./quatadd.o ./quatconj.o ./quatdot.o ./quatnorm.o ./quatnormsquared.o ./quatproduct.o ./quatsubtraction.o  ./quatadd.d ./quatconj.d ./quatdot.d ./quatnorm.d ./quatnormsquared.d ./quatproduct.d ./quatsubtraction.d  libQuaternion.a
rm -rf  ./MatchPatternToObserved.o ./Sequence.o  ./MatchPatternToObserved.d ./Sequence.d  libActivityRecognition.a
rm -rf  ./Human.o  ./Human.d  libHuman.a
rm -rf  ./XMLHumanActivityRead.o ./XMLHumanPoseRead.o  ./XMLHumanActivityRead.d ./XMLHumanPoseRead.d  libHuman_Read_XML.a
rm -rf  ./src/ReconizeHumanObserved.o  ./src/ReconizeHumanObserved.d libReconizeHumanObserved.a

export LD_LIBRARY_PATH=/Poseproject/C/libmxml:$LD_LIBRARY_PATH

#Compile
echo Quaternion
cd Quaternion/Release
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatadd.d" -MT"quatadd.d" -o"quatadd.o" "../quatadd.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatconj.d" -MT"quatconj.d" -o"quatconj.o" "../quatconj.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatdot.d" -MT"quatdot.d" -o"quatdot.o" "../quatdot.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatnorm.d" -MT"quatnorm.d" -o"quatnorm.o" "../quatnorm.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatnormsquared.d" -MT"quatnormsquared.d" -o"quatnormsquared.o" "../quatnormsquared.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatproduct.d" -MT"quatproduct.d" -o"quatproduct.o" "../quatproduct.c"
arm-linux-gnueabi-gcc -O3 -Wall -c -fmessage-length=0 -fPIC -lm -MMD -MP -MF"quatsubtraction.d" -MT"quatsubtraction.d" -o"quatsubtraction.o" "../quatsubtraction.c"
#arm-linux-gnueabi-ar -r "libQuaternion.a"  ./quatadd.o ./quatconj.o ./quatdot.o ./quatnorm.o ./quatnormsquared.o ./quatproduct.o ./quatsubtraction.o   
arm-linux-gnueabi-gcc -shared -o "libQuaternion.so"  ./quatadd.o ./quatconj.o ./quatdot.o ./quatnorm.o ./quatnormsquared.o ./quatproduct.o ./quatsubtraction.o
echo Quaternion Done


echo Activity Recognition
cd ../../ActivityRecognition/Release
arm-linux-gnueabi-gcc -I"/PoseProject/C/Quaternion" -O3 -Wall -c -fmessage-length=0 -lm -fPIC -MMD -MP -MF"MatchPatternToObserved.d" -MT"MatchPatternToObserved.d" -o"MatchPatternToObserved.o" "../MatchPatternToObserved.c"
arm-linux-gnueabi-gcc -I"/PoseProject/C/Quaternion" -O3 -Wall -c -fmessage-length=0 -lm -fPIC -MMD -MP -MF"Sequence.d" -MT"Sequence.d" -o"Sequence.o" "../Sequence.c"
#arm-linux-gnueabi-ar -r "libActivityRecognition.a"  ./MatchPatternToObserved.o ./Sequence.o   
arm-linux-gnueabi-gcc -shared -o "libActivityRecognition.so"  ./MatchPatternToObserved.o ./Sequence.o   
echo Activity Recognition Done


echo Human
cd ../../Human/Release
arm-linux-gnueabi-gcc -I"/PoseProject/C/ActivityRecognition" -I"/PoseProject/C/Quaternion" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"Human.d" -MT"Human.d" -o"Human.o" "../Human.c" -lm
#arm-linux-gnueabi-ar -r "libHuman.a"  ./Human.o   
arm-linux-gnueabi-gcc -shared -o "libHuman.so"  ./Human.o   
echo Human Done


echo Human Read XML
cd ../../Human_Read_XML/Release
arm-linux-gnueabi-gcc -I"/PoseProject/C/Quaternion" -I"/PoseProject/C/Human" -I"/PoseProject/C/ActivityRecognition" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"XMLHumanActivityRead.d" -MT"XMLHumanActivityRead.d" -o"XMLHumanActivityRead.o" "../XMLHumanActivityRead.c" -lm -lmxml 
arm-linux-gnueabi-gcc -I"/PoseProject/C/Quaternion" -I"/PoseProject/C/Human" -I"/PoseProject/C/ActivityRecognition" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"XMLHumanPoseRead.d" -MT"XMLHumanPoseRead.d" -o"XMLHumanPoseRead.o" "../XMLHumanPoseRead.c" -lm -lmxml
arm-linux-gnueabi-gcc -I"/PoseProject/C/Quaternion" -I"/PoseProject/C/Human" -I"/PoseProject/C/ActivityRecognition" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"Quaternion_Parsing.d" -MT"Quaternion_Parsing.d" -o"Quaternion_Parsing.o" "../Quaternion_Parsing.c" -lm -lmxml
#arm-linux-gnueabi-ar -r "libHuman_Read_XML.a"  ./XMLHumanActivityRead.o ./XMLHumanPoseRead.o ./Quaternion_Parsing.o
arm-linux-gnueabi-gcc -shared -o "libHuman_Read_XML.so"  ./XMLHumanActivityRead.o ./XMLHumanPoseRead.o ./Quaternion_Parsing.o
echo Human Read Xml Done


echo ReconizeHumanObserved
cd ../../ReconizeHumanObserved/Release
arm-linux-gnueabi-gcc -I"/PoseProject/C/ActivityRecognition" -I"/PoseProject/C/Human_Read_XML" -I"/PoseProject/C/Quaternion" -I"/PoseProject/C/Human" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"src/ReconizeHumanObserved.d" -MT"src/ReconizeHumanObserved.d" -o"src/ReconizeHumanObserved.o" "../src/ReconizeHumanObserved.c"
#arm-linux-gnueabi-gcc -L"/PoseProject/C/Human/Release" -L"/PoseProject/C/Human_Read_XML/Release" -L"/PoseProject/C/Quaternion/Release" -L"/PoseProject/C/ActivityRecognition/Release" -o"ReconizeHumanObserved"  ./src/ReconizeHumanObserved.o  -lHuman -lHuman_Read_XML -lActivityRecognition -lQuaternion -lmxml -pthread -lm 
#arm-linux-gnueabi-ar -r "libReconizeHumanObserved.a" "./src/ReconizeHumanObserved.o"
arm-linux-gnueabi-gcc -shared -o  "libReconizeHumanObserved.so" "./src/ReconizeHumanObserved.o" "../../Human_Read_XML/Release/XMLHumanActivityRead.o" "../../Human_Read_XML/Release/XMLHumanPoseRead.o" "../../Human_Read_XML/Release/Quaternion_Parsing.o" "../../Human/Release/Human.o" "../../ActivityRecognition/Release/MatchPatternToObserved.o" "../../ActivityRecognition/Release/Sequence.o" "/PoseProject/C/Quaternion/Release/quatadd.o" "/PoseProject/C/Quaternion/Release/quatconj.o" "/PoseProject/C/Quaternion/Release/quatdot.o" "/PoseProject/C/Quaternion/Release/quatnorm.o" "/PoseProject/C/Quaternion/Release/quatnormsquared.o" "/PoseProject/C/Quaternion/Release/quatproduct.o" "/PoseProject/C/Quaternion/Release/quatsubtraction.o" "/PoseProject/C/libmxml/libmxml.so"
echo ReconizeHumanObserved Done

echo MotionCapture
cd ../../MotionCapture
arm-linux-gnueabi-g++ -D__GCC_HAVE_SYNC_COMPARE_AND_SWAP_{1,2,4} -Wl,--no-as-needed -std=gnu++11 -pthread -I"/PoseProject/C/ReconizeHumanObserved/src" -I"/PoseProject/C/ReconizeHumanObserved/Release/src" -I"/PoseProject/C/ActivityRecognition" -I"/PoseProject/C/Human_Read_XML" -I"/PoseProject/C/Quaternion" -I"/PoseProject/C/Human" -L"/PoseProject/C/Human/Release" -L"/PoseProject/C/Human_Read_XML/Release" -L"/PoseProject/C/Quaternion/Release" -L"/PoseProject/C/ActivityRecognition/Release" -L "/PoseProject/C/ReconizeHumanObserved/Release" -o ../APPLICATION main.cpp Matrix.cpp MotionCapture.cpp Kalman.cpp SuperKalman.cpp Quaternion.cpp QVector.cpp -lm -lmxml -lHuman -lHuman_Read_XML -lActivityRecognition -lQuaternion -lReconizeHumanObserved #-lbluetooth

echo APPLICATION created

echo Copying files...

#copy all the shared libraries(.so files) to import them into the lib of running environment. Here the destination is according to my system.
cd /PoseProject/C
cp APPLICATION EXECUT/
cp ReconizeHumanObserved/Release/libReconizeHumanObserved.so LIBS/
cp Human_Read_XML/Release/libHuman_Read_XML.so LIBS/
cp Quaternion/Release/libQuaternion.so LIBS/
cp ActivityRecognition/Release/libActivityRecognition.so LIBS/
cp Human/Release/libHuman.so LIBS/

echo Done


