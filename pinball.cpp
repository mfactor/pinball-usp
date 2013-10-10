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

  // histogram
  FILE *xymap = fopen("xymap.dat", "w");
  FILE *hist = fopen("histogram.dat", "w");
  FILE *gnuplot = popen("gnuplot", "w");
  fprintf(gnuplot, "reset\n");
  fprintf(gnuplot, "binwidth=5\n");
  fprintf(gnuplot, "bin(x,width)=width*floor(x/width)\n");
  fprintf(gnuplot, "plot sin(x)\n");
  fflush(gnuplot);

  // fps
  time_t start, end;
  int count;
  // keyboard action
  int key;

  //int resW = 1280, resH = 720;
  int resW = 640, resH = 480;
  //int resW = 320, resH = 176;

  VideoCapture cap(1); // 0=default, 1=usb
  if(!cap.isOpened())
    return -1;

  printf("W = %g, H = %g\n", cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));

  cap.set(CV_CAP_PROP_FRAME_WIDTH, resW);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, resH);

  resW = (int) cap.get(CV_CAP_PROP_FRAME_WIDTH);
  resH = (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT);

  printf("W = %i, H = %i\n", resW, resH);

  

  Scalar green = Scalar( 0, 255, 0 );
  Scalar red = Scalar(0, 0, 255);
  Scalar blue = Scalar(255, 0, 0);
  Scalar yellow = Scalar(0, 255, 255);

  Mat frame, greenpts, redpads, warp, newf, oldf, delta, drawgreen, drawpads;
  Mat yellowgoal, drawyellow;
  Mat track;
  Mat M;
  // opens a window 
  namedWindow("WebCam", 0);
  namedWindow("Motion", 0);
  namedWindow("Warp", 0);
  //warp = imread("notfound.png", 1);
  //delta = imread("notfound.png", 1);

  track.create(resH, resW, CV_8UC3);
  greenpts.create(resH, resW, CV_8UC3);
  newf.create(resH, resW, CV_8UC3);
  oldf.create(resH, resW, CV_8UC3);
  yellowgoal.create(resH, resW, CV_8UC3);

  warp.create(resH, resW, CV_8UC3);
  delta.create(resH, resW, CV_8UC3);
  drawyellow.create(resH, resW, CV_8UC3);
  drawgreen.create(resH, resW, CV_8UC3);
  drawpads.create(resH, resW, CV_8UC3);



  int paused = 1; // starts paused 



  // track
  int
    posX = -1,
    posY = -1,
    lastX = -1,
    lastY = -1;
  double vx, vy;
  double ax, ay, dt;
  double x0, y0, xt, yt;
  double trackpos[2][10];
  double trackV[2][9];
  double trackA[2][8];
  double avgV[2], avgA[2];
  Point2f bola, bola_adv;
  double tshift_adv;
  bola.x = -1;
  bola.y = -1;
  int yhist = 0; // count histogram only above this value

  // parameters to find green points
  int findgreen = TRUE;
  int 
    LH = 40, // h
    LS = 110, // s
    LV = 0, // v
    HH = 80, // h
    HS = 255, // s
    HV = 255; // v

  // parameters to find red pads
  int findred = TRUE;
  vector<vector<Point> > padpolys(2);
  double padA[2], padB[2]; // pad size sides
  padA[0] = padA[1] = padB[0] = padB[1] = 1.0;
  int 
    LH2 = 0, // h
    LS2 = 100, // s
    LV2 = 0, // v
    HH2 = 10, // h
    HS2 = 255, // s
    HV2 = 255; // v

  // find yellow obstacle
  double goalx, goaly, goalr;
  int findyellow = TRUE;
  int 
    LH3 = 24, // h
    LS3 = 200, // s
    LV3 = 0, // v
    HH3 = 30, // h
    HS3 = 255, // s
    HV3 = 255; // v


  // create trackbars
  int **bars = (int **) malloc(6*sizeof(int *));
  bars[0] = &LH3;
  bars[1] = &HH3;
  bars[2] = &LS3;
  bars[3] = &HS3;
  bars[4] = &LV3;
  bars[5] = &HV3;
  Mat binimg;
  namedWindow("Binary", 0);
  createTrackbar("low H", "Binary", bars[0], 255, NULL, 0);
  createTrackbar("high H", "Binary", bars[1], 255, NULL, 0);
  createTrackbar("low S", "Binary", bars[2], 255, NULL, 0);
  createTrackbar("high S", "Binary", bars[3], 255, NULL, 0);
  createTrackbar("low V", "Binary", bars[4], 255, NULL, 0);
  createTrackbar("high V", "Binary", bars[5], 255, NULL, 0); 


  // pads position
  int xpadcenter[2], ypadcenter[2];
  int padR[2];
  xpadcenter[0] = xpadcenter[1] = -1;
  ypadcenter[0] = ypadcenter[1] = -1;
  padR[0] = padR[1] = -1;
  int delayA = 0, delayD = 0;
  int hit = FALSE;


  // shows the video until key 'q' is pressed
  count=0;
  bool first_frame = true;
  bool Mdef = false;
  key = 's';
  time(&start);
  while (key != 'q') {
    cap >> frame; 

    cvtColor(frame, binimg, CV_BGR2HSV);
    inRange(binimg, Scalar(*bars[0], *bars[2], *bars[4]), Scalar(*bars[1], *bars[3], *bars[5]), binimg);
    imshow("Binary", binimg); // update image


    // square projection
    if (findgreen) {

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
	newedge[1].x = (double) resW; newedge[1].y = 000.0;
	newedge[2].x = (double) resW; newedge[2].y = (double) resH;
	newedge[3].x = 000.0; newedge[3].y = (double) resH;

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
	findgreen = FALSE;

      } else {// if != 4
	warp.setTo(Scalar(0,0,0)); // clear = imread("notfound.png", 1);
	delta.setTo(Scalar(0,0,0)); // clear = imread("notfound.png", 1);
      }

    } // if green

    if (Mdef) {
      warpPerspective(frame, warp, M, frame.size(), INTER_NEAREST, BORDER_CONSTANT, 0);

      if (findred) {

	drawpads.setTo(Scalar(0,0,0)); // clear

	// find pads
	cvtColor(warp, redpads, CV_BGR2HSV);
	inRange(redpads, Scalar(LH2, LS2, LV2), Scalar(HH2, HS2, HV2), redpads);
	GaussianBlur(redpads, redpads, Size(7,7), 0, 0);
    
	vector<vector<Point> > padcontours;
	vector<Vec4i> padhierarchy;
	findContours(redpads, padcontours, padhierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
	vector<vector<Point> > padcontours_poly( padcontours.size() );
	vector<Point2f> padcenter( padcontours.size() );
	vector<float> padradius( padcontours.size() );
	for(int i = 0; i < padcontours.size(); i++ ) {
	  approxPolyDP( Mat(padcontours[i]), padcontours_poly[i], 3, true);
	  minEnclosingCircle( (Mat) padcontours_poly[i], padcenter[i], padradius[i]);
	}

	if (padcontours.size() == 2) {

	  padR[0] = padR[1] = (int) ( (padradius[0] + padradius[1])/2.0 );
	  yhist = (int) ( (padcenter[0].y+padcenter[1].y)/2.0 + 6.0*padR[0] );

	  if (padcenter[0].x > padcenter[1].x) {
	    xpadcenter[0] = padcenter[0].x;
	    ypadcenter[0] = padcenter[0].y;

	    xpadcenter[1] = padcenter[1].x;
	    ypadcenter[1] = padcenter[1].y;
	  } else {
	    xpadcenter[1] = padcenter[0].x;
	    ypadcenter[1] = padcenter[0].y;

	    xpadcenter[0] = padcenter[1].x;
	    ypadcenter[0] = padcenter[1].y;
	  }

	  findred = FALSE;
	}

      } // if red

      // find yellow (real time)
      if (findyellow) {
	drawyellow.setTo(Scalar(0,0,0)); // clear

	// find pads
	cvtColor(warp, yellowgoal, CV_BGR2HSV);
	inRange(yellowgoal, Scalar(LH3, LS3, LV3), Scalar(HH3, HS3, HV3), yellowgoal);
	GaussianBlur(yellowgoal, yellowgoal, Size(7,7), 0, 0);
    
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(yellowgoal, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > contours_poly( contours.size() );
	vector<Point2f> center( contours.size() );
	vector<float> radius( contours.size() );
	for(int i = 0; i < contours.size(); i++ ) {
	  approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true);
	  minEnclosingCircle( (Mat) contours_poly[i], center[i], radius[i]);
	  circle(drawyellow, center[i], (int)radius[i]/2, yellow, 2, 8, 0 );
	}
	  	
	if (contours.size() == 1) {
	  goalx = center[0].x;
	  goaly = center[0].y;
	  goalr = radius[0];
	} else {
	  goalr = -1.0;
	}
	
      } // if findyellow


      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================


      // pads area sizes
      // raio do objeto amarelo: rem objeto => goalr = -1
      if (goalr > 0) {
	double max = 2.0, min = 0.5;
	double LR = goalx/((double) resW); // 0 .. 1
   // goalx - posição do centro do objeto amarelo

	// left pad lateral sides
	padA[0] = min + fabs(0.0-LR)*(max-min);
	padB[0] = min + fabs(0.0-LR)*(max-min);

	// right pad lateral sides
	padA[1] = min + fabs(1.0-LR)*(max-min);
	padB[1] = min + fabs(1.0-LR)*(max-min);
      } else {
	padA[0] = padB[0] = padA[1] = padB[1] = 1.0;
      }


      // pads polygon
      drawpads.setTo(Scalar(0,0,0)); // clear

      vector<Point> aux0, aux1;
      Point p1, p2, p3, p4;
      int i;
      double alpha3x,alpha4x; // alpha for p3 and alpha for p4
      double alpha3y,alpha4y; // alpha for p3 and alpha for p4

      // left and right looking at camera
      // ball at center => alpha3x = alpha4x = 1.0
      // ball at right  => alpha3x = -1.0, alpha4x = 3.0
      // ball at left   => alpha3x = 3.0, alpha4x = -1.0
      alpha3x = 0.0 + 2.0*posX/resW;
      alpha4x = 2.0 - 2.0*posX/resW;

      alpha3y = alpha4y = 1.0 + 2.0*abs(vy)/30.0;

      printf("%f \n",vy);
    
      // i=0 => left pad (olhando pra camera)
      //---------- 
      i=0;
      p1 = Point(xpadcenter[i]-padR[i], ypadcenter[i]-padR[i]/2.0);
      p2 = Point(xpadcenter[i]+padR[i], ypadcenter[i]-padR[i]/2.0);
      p3 = Point(xpadcenter[i]+alpha3x*padR[i], ypadcenter[i]+alpha3y*padR[i]*padA[i]);
      p4 = Point(xpadcenter[i]-alpha4x*padR[i], ypadcenter[i]+alpha4y*padR[i]*padB[i]);
      
      // ordem dos pontos (camera na parte de cima da mesa):
      //
      //     \o - right pad
      //
      // p1------p2
      // |       |
      // |       |
      // |       |
      // p4------p3
      //
      //    \  /
      //     C  - camera

      aux0.push_back(p1);
      aux0.push_back(p2);
      aux0.push_back(p3);
      aux0.push_back(p4);
      padpolys[i] = aux0;

      line(drawpads, p1, p2, red, 2);
      line(drawpads, p2, p3, red, 2);
      line(drawpads, p3, p4, red, 2);
      line(drawpads, p4, p1, red, 2);
      //----------



      // i=1 => right pad  (olhando pra camera)
      //----------
      i=1;
      p1 = Point(xpadcenter[i]-padR[i], ypadcenter[i]-padR[i]/2.0);
      p2 = Point(xpadcenter[i]+padR[i], ypadcenter[i]-padR[i]/2.0);
      p3 = Point(xpadcenter[i]+alpha3x*padR[i], ypadcenter[i]+alpha3y*padR[i]*padA[i]);
      p4 = Point(xpadcenter[i]-alpha4x*padR[i], ypadcenter[i]+alpha4y*padR[i]*padB[i]);
      
      // ordem dos pontos (camera na parte de cima da mesa):
      //
      //     \o - right pad
      //
      // p1------p2
      // |       |
      // |       |
      // |       |
      // p4------p3
      //
      //    \  /
      //     C  - camera

      aux1.push_back(p1);
      aux1.push_back(p2);
      aux1.push_back(p3);
      aux1.push_back(p4);
      padpolys[i] = aux1;

      line(drawpads, p1, p2, red, 2);
      line(drawpads, p2, p3, red, 2);
      line(drawpads, p3, p4, red, 2);
      line(drawpads, p4, p1, red, 2);
      //----------

      // edge for the histogram
      line(drawpads, Point(0, yhist), Point(resW, yhist), green, 1);


      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================
      // ==========================================================




      // motion
      cvtColor(warp, newf, CV_BGR2GRAY);
      if (first_frame) {
	oldf = newf.clone();
	delta.setTo(Scalar(0,0,0)); // clear = imread("notfound.png", 1);
	first_frame = false;
      } else {
	absdiff(newf, oldf, delta);
	threshold(delta, delta, 5, 255, THRESH_BINARY);
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
      // I set the positions to negative whenever the code does not detect it properly:
      // + only calculates the velocity if two consecutive measurements return proper positions
      posX = posY = -1.0;
      if (norm > 0) {
	posX = sumX/norm;
	posY = sumY/norm;
      }
      if (posX < 0 || posY < 0 || posX > resW || posY > resH) {
	posX = posY = -1.0;
	bola.x = bola.y = -1.0;
      }
      vx = vy = 0.0;
      if (bola.x > 0.0) {
	vx = posX - bola.x;
	vy = posY - bola.y;
      }
      bola.x = posX;
      bola.y = posY;
      // time shift to fix ball position:
      // + fix capture delay 
      // + advanced motion to fix pad action delay
      tshift_adv = 2.0;
      bola_adv.x = bola.x + vx*tshift_adv;
      bola_adv.y = bola.y + vy*tshift_adv;
      // ---
      circle(warp, bola_adv, 10, blue, 2, 8, 0);


      // HISTOGRAM: REAL TIME DATA
      // + only if ball is detected within boundaries

      if (!paused && bola.x > 0 && bola.x < resW && bola.y > yhist && bola.y < resH) {
      	fprintf(xymap, "%g %g %g %g\n", bola.x, bola.y, vx, vy);
      	fflush(xymap);

      	//fprintf(gnuplot, "plot 'xymap.dat' u 1:2 w lp\n");
	fprintf(gnuplot, "plot 'xymap.dat' using (bin($1,binwidth)):(1.0) smooth freq with boxes t 'histogram'\n");
      	fflush(gnuplot);
      }


      // HISTOGRAM: ONLY DATA COLLECTED AFTER PAD ACTION
      // + only if ball is detected within boundaries
      // + only after hit = TRUE
      // + only after pads return to stationary position (delays)
      // if (hit == TRUE && vy < 0 && (delayA > 10 && delayD > 10) && bola.x > 0 && bola.x < resW && bola.y > 0 && bola.y < resH) {
      // 	//&& fabs(vy) + fabs(vx) < 100
      // 	fprintf(hist, "%g %g %g %g\n", vx, vy, bola_adv.x, bola_adv.y);
      // 	fflush(hist);
      // 	fprintf(gnuplot, "plot 'histogram.dat' using (bin($1,binwidth)):(1.0) smooth freq with boxes t 'histogram'\n");
      // 	fflush(gnuplot);
      // 	hit = FALSE;
      // }

    } // if Mdef

    add(drawgreen, frame, frame);
    add(drawpads, warp, warp);
    add(drawyellow, warp, warp);
    if (paused)
      putText(warp, "paused", Point(resW/2, resH/2), FONT_HERSHEY_SIMPLEX, 1.0, red);

    line(delta, Point(0, yhist), Point(resW, yhist), Scalar(255,255,255), 1);

    
    // aceleracao
    if (bola.x > 0) {

      for (int i=9; i>0; i--) { // fifo
	trackpos[0][i] = trackpos[0][i-1]; // x
	trackpos[1][i] = trackpos[1][i-1]; // y
      }
      trackpos[0][0] = bola.x;
      trackpos[1][0] = bola.y;
      int dataok = TRUE; // data is ok if all frames have position well defined
      for (int i=0; i<10; i++)
	if (trackpos[0][i] < 0 || trackpos[1][i] < 0)
	  dataok = FALSE;
      
      if (dataok) {
	// printf("POS: ");
	// for (int i=0; i<10; i++)
	//   printf("(%g, %g) --", trackpos[0][i], trackpos[1][i]);
	// printf("\n");
	
	// printf("SPD: ");
	for (int i=0; i<9; i++) {
	  trackV[0][i] = trackpos[0][i]-trackpos[0][i+1];
	  trackV[1][i] = trackpos[1][i]-trackpos[1][i+1];
	  // printf("(%g, %g) --", trackV[0][i], trackV[1][i]);
	}
	// printf("\n");
	
	// printf("ACE: ");
	for (int i=0; i<8; i++) {
	  trackA[0][i] = trackV[0][i]-trackV[0][i+1];
	  trackA[1][i] = trackV[1][i]-trackV[1][i+1];
	  // printf("(%g, %g) --", trackA[0][i], trackA[1][i]);
	}
	// printf("\n\n\n");
	
      } // if dataok
            

      if (key == 't' || TRUE) {
	track.setTo(Scalar(0,0,0)); // clear

	avgA[0] = 0.0;
	avgA[1] = -0.8; // g sin(theta) unidades de pixel/frame^2
	avgV[0] = vx;
	avgV[1] = vy;
	// if (dataok) {
	//   avgV[0] = avgV[1] = 0.0;
	//   for (int i=0; i<9; i++) {
	//     avgV[0] += trackV[0][i]/9.0;
	//     avgV[1] += trackV[1][i]/9.0;
	//   }
	//   avgA[0] = avgA[1] = 0.0;
	//   for (int i=0; i<8; i++) {
	//     avgA[0] += trackA[0][i]/8.0;
	//     avgA[1] += trackA[1][i]/8.0;
	//   }
	// }

	dt = 0; // init
	x0 = xt = bola.x;
	y0 = yt = bola.y;
	while (xt > 0 && xt < resW && yt > 0 && yt < resH) {
	  dt += 1E-2; // time step
	  x0 = xt;
	  y0 = yt;
	  xt = bola.x + avgV[0]*dt + 0.5*avgA[0]*dt*dt;
	  yt = bola.y + avgV[1]*dt + 0.5*avgA[1]*dt*dt;
	  
	  // draw:
	  line(track, Point(x0, y0), Point(xt, yt), blue, 2);
	  //circle(track, Point(xt, yt), 5, blue, 1);
	}
      }
    }
    add(track, warp, warp);


    imshow("Motion", delta);
    imshow("Warp", warp);
    imshow("WebCam", frame);

    // calculate fps
    count++;
    if (count == 100) {
      time(&end);
      //printf("FPS: %g\n", count/difftime(end, start));
      count = 0;
      time(&start);
    }

    key = waitKey(1); // milliseconds

    
    // check ball position
    if (!findred) {
      double distA = pointPolygonTest(padpolys[0], bola_adv, false);
      double distB = pointPolygonTest(padpolys[1], bola_adv, false);

      if (distA > 0)
	key = 'a';
      if (distB > 0)
	key = 'd';      
    }




    // WASD control of pads
    switch(key) {
    case 'p':
      paused = 1-paused; // anternates between 0 and 1
      break;
    case 'a':
      if (delayA > 15 && !paused) {
	maestroSetTarget(fd, 0, 6000);
	usleep(50000);
	maestroSetTarget(fd, 0, 1000);
	delayA = delayD = 0;
	hit = TRUE;
      }
      break;
    case 'd':
      if (delayD > 15 && !paused) {
	maestroSetTarget(fd, 1, 6000);
	usleep(50000);
	maestroSetTarget(fd, 1, 1000);
	delayA = delayD = 0;
	hit = TRUE;
      }
      break;
    case 'r':
      findred = TRUE;
      break;
    case 'g':
      findgreen = TRUE;
      break;
    case 'h':
      // clear histogram
      fclose(hist);
      fclose(xymap);
      xymap = fopen("xymap.dat", "w");
      hist = fopen("histogram.dat", "w");
      fprintf(gnuplot, "plot sin(x)\n");
      fflush(gnuplot);
      break;
    case '1':
      bars[0] = &LH;
      bars[1] = &HH;
      bars[2] = &LS;
      bars[3] = &HS;
      bars[4] = &LV;
      bars[5] = &HV;
      break;
    case '2':
      bars[0] = &LH2;
      bars[1] = &HH2;
      bars[2] = &LS2;
      bars[3] = &HS2;
      bars[4] = &LV2;
      bars[5] = &HV2;
      break;
    case '3':
      bars[0] = &LH3;
      bars[1] = &HH3;
      bars[2] = &LS3;
      bars[3] = &HS3;
      bars[4] = &LV3;
      bars[5] = &HV3;
      break;
    }
    delayA++;
    delayD++;

    if (key == '1' || key == '2' || key == '3') {
      destroyWindow("Binary");
      namedWindow("Binary", 0);
      createTrackbar("low H", "Binary", bars[0], 255, NULL, 0);
      createTrackbar("high H", "Binary", bars[1], 255, NULL, 0);
      createTrackbar("low S", "Binary", bars[2], 255, NULL, 0);
      createTrackbar("high S", "Binary", bars[3], 255, NULL, 0);
      createTrackbar("low V", "Binary", bars[4], 255, NULL, 0);
      createTrackbar("high V", "Binary", bars[5], 255, NULL, 0); 
    }

  } // loop until q

  // turn off pads
  maestroSetTarget(fd, 0, 1000);
  maestroSetTarget(fd, 1, 1000);
  close(fd);
  fclose(hist);
  pclose(gnuplot);
  fclose(xymap);

  return(0);
}
