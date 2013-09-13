#include <iostream>
#include <cmath>
#include <ctime>

// USB pololu
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// opencv
#include <cv.h>
#include <highgui.h>
using namespace cv;

using namespace std;

// --- USB pololu
int maestroSetTarget(int fd, unsigned char channel, unsigned short target) {
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1) {
    perror("error writing");
    return(-1);
  }
  return(0);
}
// ---


int main(int argc, char *argv[]) {
  
  // USB pololu
  const char *device = "/dev/ttyACM0";  
  //const char * device = "/dev/ttyACM0";  // Linux
  //const char * device = "/dev/cu.usbmodem00034567"; // Mac OS X
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1) {
    perror(device);
    return(1);
  }



  // fps
  time_t start, end;
  int count;
  // keyboard action
  int key;


  VideoCapture cap(1); // 0=default, 1=usb
  if(!cap.isOpened())
    return -1;
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  // how to turn off auto-focus?????
  Scalar green = Scalar( 0, 255, 0 );
  Scalar red = Scalar(0, 0, 255);
  Scalar blue = Scalar(255, 0, 0);

  Mat frame, greenpts, redpads, warp, newf, oldf, delta, drawgreen, drawpads;
  Mat M;
  // opens a window 
  namedWindow("WebCam", 0);
  //namedWindow("Binary", 0);
  namedWindow("Warp", 0);
  namedWindow("Motion", 0);
  warp = imread("notfound.png", 1);
  delta = imread("notfound.png", 1);
  drawgreen.create(480, 640, CV_8UC3);
  drawpads.create(480, 640, CV_8UC3);

  // track
  int
    posX = -1,
    posY = -1,
    lastX = -1,
    lastY = -1;

  // parameters to find green points
  int 
    LH = 40, // h
    LS = 110, // s
    LV = 0, // v
    HH = 80, // h
    HS = 255, // s
    HV = 255; // v

  // parameters to find red pads
  int 
    LH2 = 0, // h
    LS2 = 70, // s
    LV2 = 0, // v
    HH2 = 7, // h
    HS2 = 255, // s
    HV2 = 255; // v

  // create trackbars
  // createTrackbar("low H", "Binary", &LH2, 255, NULL, 0);
  // createTrackbar("high H", "Binary", &HH2, 255, NULL, 0);
  // createTrackbar("low S", "Binary", &LS2, 255, NULL, 0);
  // createTrackbar("high S", "Binary", &HS2, 255, NULL, 0);
  // createTrackbar("low V", "Binary", &LV2, 255, NULL, 0);
  // createTrackbar("high V", "Binary", &HV2, 255, NULL, 0);


  // pads position
  int xpadcenter[2], ypadcenter[2];
  int padR[2];
  xpadcenter[0] = xpadcenter[1] = -1;
  ypadcenter[0] = ypadcenter[1] = -1;
  padR[0] = padR[1] = -1;
  int delayA = 0, delayD = 0;


  // shows the video until key 'q' is pressed
  count=0;
  bool first_frame = true;
  bool Mdef = false;
  key = 's';
  time(&start);
  while (key != 'q') {
    cap >> frame; 

    // square projection
    if (count == 99) {

      drawgreen.setTo(Scalar(0,0,0)); // clear

      // find green points
      cvtColor(frame, greenpts, CV_BGR2HSV);
      inRange(greenpts, Scalar(LH, LS, LV), Scalar(HH, HS, HV), greenpts);

      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      findContours(greenpts, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

      vector<vector<Point> > contours_poly( contours.size() );
      vector<Point2f>center( contours.size() );
      vector<float>radius( contours.size() );
      for(int i = 0; i < contours.size(); i++ ) {
	approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
	minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i]);
	circle(drawgreen, center[i], (int)radius[i], green, 2, 8, 0 );
      }

      if (contours.size() == 4) {

	vector<Point2f> newedge(4);
	newedge[0].x = 000.0; newedge[0].y = 000.0;
	newedge[1].x = 640.0; newedge[1].y = 000.0;
	newedge[2].x = 640.0; newedge[2].y = 480.0;
	newedge[3].x = 000.0; newedge[3].y = 480.0;

	vector<Point2f> oldedge(4);

	double avx=0.0, avy=0.0;
	double angles[4];
	int sort[4];
	for (int i=0; i<4; i++) {
	  avx += center[i].x/4.0;
	  avy += center[i].y/4.0;
	}
	for (int i=0; i<4; i++) {
	  angles[i] = atan2(center[i].y-avy, center[i].x-avx)+M_PI;
	  sort[i] = i;
	}

	int swap=0;
	while (swap == 0) {
	  swap = 1;
	  for (int i=0; i<3; i++) {	  
	    if (angles[sort[i]] > angles[sort[i+1]]) {
	      int ii = sort[i];
	      sort[i] = sort[i+1];
	      sort[i+1] = ii;
	      swap = 0;
	    }	  
	  }
	}
	for (int i=0; i<4; i++) {
	  oldedge[i].x = center[sort[i]].x;
	  oldedge[i].y = center[sort[i]].y;
	}

	M = getPerspectiveTransform(oldedge, newedge);
	Mdef = true;

      } else {// if != 4
	warp = imread("notfound.png", 1);
	delta = imread("notfound.png", 1);
      }

    } // count 100

    if (Mdef) {
      warpPerspective(frame, warp, M, frame.size(), INTER_NEAREST, BORDER_CONSTANT, 0);


      if (count == 99) {

	drawpads.setTo(Scalar(0,0,0)); // clear

	// find pads
	cvtColor(warp, redpads, CV_BGR2HSV);
	inRange(redpads, Scalar(LH2, LS2, LV2), Scalar(HH2, HS2, HV2), redpads);
	GaussianBlur(redpads, redpads, Size(7,7), 0, 0);
	//imshow("Binary", redpads); // update image
    
	vector<vector<Point> > padcontours;
	vector<Vec4i> padhierarchy;
	findContours(redpads, padcontours, padhierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
	vector<vector<Point> > padcontours_poly( padcontours.size() );
	vector<Point2f> padcenter( padcontours.size() );
	vector<float> padradius( padcontours.size() );
	for(int i = 0; i < padcontours.size(); i++ ) {
	  approxPolyDP( Mat(padcontours[i]), padcontours_poly[i], 3, true);
	  minEnclosingCircle( (Mat) padcontours_poly[i], padcenter[i], padradius[i]);
	  circle(drawpads, padcenter[i], (int)padradius[i], red, 2, 8, 0);
	}

	if (padcontours.size() == 2) {
	  xpadcenter[0] = padcenter[0].x;
	  ypadcenter[0] = padcenter[0].y;
	  padR[0] = padradius[0];

	  xpadcenter[1] = padcenter[1].x;
	  ypadcenter[1] = padcenter[1].y;
	  padR[1] = padradius[1];
	}

      }



      // motion
      cvtColor(warp, newf, CV_BGR2GRAY);
      if (first_frame) {
	oldf = newf.clone();
	delta = imread("notfound.png", 1);
	first_frame = false;
      } else {
	absdiff(newf, oldf, delta);
	threshold(delta, delta, 10, 255, THRESH_BINARY);
	erode(delta, delta, Mat(), Point(-1,-1), 3);
	dilate(delta, delta, Mat(), Point(-1,-1), 1);

	oldf = newf.clone();
      }

      // track
      double norm = 0.0, sumX = 0.0, sumY = 0.0, sizex = delta.cols, sizey = delta.rows;
      for(int i = 0; i < sizex; i++) {
	for(int j = 0; j < sizey; j++) {
	  norm += ((double) delta.data[j*delta.step+i]);
	  sumX += i*((double) delta.data[j*delta.step+i]);
	  sumY += j*((double) delta.data[j*delta.step+i]);
	}
      }
      posX = sumX/norm;
      posY = sumY/norm;
      Point2f bola;
      bola.x = posX;
      bola.y = posY;
      circle(warp, bola, 10, blue, 2, 8, 0);
    }

    add(drawgreen, frame, frame);
    add(drawpads, warp, warp);

    imshow("Motion", delta);
    imshow("Warp", warp);
    imshow("WebCam", frame);

    // calculate fps
    count++;
    if (count == 100) {
      time(&end);
      printf("FPS: %g\n", count/difftime(end, start));
      count = 0;
      time(&start);
    }

    key = waitKey(10); // milliseconds

    
    // check ball position
    double distA = sqrt( pow(posX-xpadcenter[0], 2.0) + pow(posY-ypadcenter[0], 2.0) );
    double distD = sqrt( pow(posX-xpadcenter[1], 2.0) + pow(posY-ypadcenter[1], 2.0) );
    
    if (distA <= padR[0])
      key = 'a';
    else if (distD <= padR[1])
      key = 'd';
    

    // WASD control of pads
    if (key == 'a' && delayA > 10) {
      maestroSetTarget(fd, 0, 6000);
      usleep(50000);
      maestroSetTarget(fd, 0, 1000);
      delayA = 0;
    }
    if (key == 'd' && delayD > 10) {
      maestroSetTarget(fd, 1, 6000);
      usleep(50000);
      maestroSetTarget(fd, 1, 1000);
      delayD = 0;
    }
    delayA++;
    delayD++;


  } // loop until q

  // turn off pads
  maestroSetTarget(fd, 0, 1000);
  maestroSetTarget(fd, 1, 1000);
  close(fd);

  return(0);
}
