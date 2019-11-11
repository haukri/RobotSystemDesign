#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <iostream>

using namespace cv;
using namespace std;

Mat image_blue;
int thresh = 100;
RNG rng(12345);
void thresh_callback(int, void* );

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
  vector<int> order = {2, 1, 1};
  Mat image, image_hsv, image_red, image_yellow;
  image = imread("/home/lasse/Desktop/lego.png", CV_LOAD_IMAGE_COLOR);

  // Convert from BGR to HSV colorspace
  cvtColor(image, image_hsv, COLOR_BGR2HSV);
  // Detect the object based on HSV Range Values
  inRange(image_hsv, Scalar(52, 72, 0), Scalar(118, 255, 255), image_blue);
  inRange(image_hsv, Scalar(0, 72, 0), Scalar(15, 255, 255), image_red);
  inRange(image_hsv, Scalar(16, 72, 0), Scalar(36, 255, 255), image_yellow);

  Mat canny_blue, canny_yellow, canny_red;
  vector<vector<Point> > contours_blue, contours_yellow, contours_red;
  vector<Vec4i> hierarchy_blue, hierarchy_yellow, hierarchy_red;
  Canny( image_blue, canny_blue, 100, 200 );
  findContours(canny_blue, contours_blue, hierarchy_blue, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

  Canny( image_red, canny_red, 100, 200 );
  findContours(canny_red, contours_red, hierarchy_red, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

  Canny( image_yellow, canny_yellow, 100, 200 );
  findContours(canny_yellow, contours_yellow, hierarchy_yellow, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );

  bool result = true;

  
  //Count Blue
  if (order[0] == contours_blue.size()) {
    //Check if they are small
    for( size_t i = 0; i< contours_blue.size(); i++ ) {
      cout << "blue: " << contourArea(contours_blue[i]) << endl;
      if (contourArea(contours_blue[i])>30000)
        result = false;
    }
  }
  else
    result = false;
  //Count Red
  if (order[1] != contours_red.size()) {
    //Check if they are normal
    for( size_t i = 0; i< contours_red.size(); i++ ) {
      if (contourArea(contours_red[i])>60000)
        result = false;
    }
  }
  else
    result = false;
  //Count Yellow
  if (order[2] != contours_yellow.size()) {
    //Check if they are normal
    for( size_t i = 0; i< contours_yellow.size(); i++ ) {
      if (contourArea(contours_yellow[i])>60000)
        result = false;
    }
  }
  else
    result = false;

  cout << "Result: " << result << endl;
  /*
  const char* source_window = "Source";
  namedWindow( source_window );
  imshow( source_window, image );
  const int max_thresh = 255;
  createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );
  waitKey();
  */
  //ros::spin();
  return 0;
}

void thresh_callback(int, void* )
{
    Mat canny_output;
    Canny( image_blue, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", drawing );
}
