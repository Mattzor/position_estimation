g++ -o test fpstestLD.cpp utils/sources/posest.cpp `pkg-config opencv --cflags --libs` -fopenmp -std=gnu++14
g++ -o testold fpstestold.cpp  `pkg-config opencv --cflags --libs`  -std=gnu++14
