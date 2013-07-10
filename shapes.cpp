#include <iostream>
#include <cmath>
#include <ctime>
#include <cv.h>
#include <highgui.h>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
  // fps
  time_t start, end;
  int count;

  // open the camera
  VideoCapture cap(0); // 0=default, 1=usb
  if(!cap.isOpened())
    return -1;
  // set the image resolution
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  // cap.set(CV_CAP_PROP_FPS, 30); // my camera does not allow me to change the fps

  // where the frames are stored (matrix)
  Mat frame, binary, mataux;

  // keyboard action
  int key;
  int imgorcam = -1;


  // opens a window 
  namedWindow("Camera", 0); // window named "Pinball" set to free size(0)
  namedWindow("Binary", 0);


  // for trackbar (default set to track orange object)
  int 
    low1 = 0, // h
    high1 = 200, // h
    low2 = 125, // s
    high2 = 255, // s
    low3 = 0, // v
    high3 = 255; // v

  // create trackbars
  createTrackbar("low H", "Binary", &low1, 255, NULL, 0);
  createTrackbar("high H", "Binary", &high1, 255, NULL, 0);
  createTrackbar("low S", "Binary", &low2, 255, NULL, 0);
  createTrackbar("high S", "Binary", &high2, 255, NULL, 0);
  createTrackbar("low V", "Binary", &low3, 255, NULL, 0);
  createTrackbar("high V", "Binary", &high3, 255, NULL, 0);




  // shows the video until key 'q' is pressed
  count=0;
  time(&start);
  for (;;) {
    if (imgorcam == -1) {
      cap >> frame; // captures the new frame from camera
      flip(frame, frame, 1); // mirror (useful if testing with webcam)
    } else 
      frame = imread("shapes.png", 1); // captures from test image

    // keyboard actions
    key = waitKey(10); // milliseconds
    if (!frame.data || key == 'q')
      break; // breaks if there's no data, or any key is pressed
    switch (key) {
    case 't':
      imgorcam *= -1; // change from test image and webcam
      break;
    }


    // apply threshold
    if (imgorcam == -1){
      cvtColor(frame, binary, CV_BGR2HSV);
      GaussianBlur(binary, binary, Size(5,5), 0, 0);
      inRange(binary, Scalar(low1, low2, low3), Scalar(high1, high2, high3), binary);
    } else {
      cvtColor(frame, binary, CV_BGR2GRAY);
    }

    // convert to grayscale
    binary.copyTo(mataux);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mataux, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    for( int i = 0; i< contours.size(); i++ ) {
      Scalar color = Scalar( 0, 0, 255 );
      drawContours( frame, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }



    // show on window
    imshow("Camera", frame); // update image
    imshow("Binary", binary);

    // calculate fps
    count++;
    if (count == 100) {
      time(&end);
      printf("FPS: %g\n", count/difftime(end, start));
      count = 0;
      time(&start);
    }

  }


  return(0);
}
