g++ -o test testsources/testSetROI.cpp testsources/testSetROIColorImageGray.cpp testsources/testSetROIColorImageBin.cpp ../sources/posest.cpp utils/assert.cpp tests.cpp `pkg-config opencv --cflags --libs`  -std=gnu++11
./test
rm test
