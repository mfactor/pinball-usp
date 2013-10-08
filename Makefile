

CVLIBS=`pkg-config --libs opencv`
CVINCS=-I/usr/include/opencv  

perspective: pinball.cpp
	g++ -O2 -o pinball pinball.cpp $(CVINCS) $(CVLIBS) -lm
