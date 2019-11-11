#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;
using namespace std;

const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

bool check_image(Mat image, string color) {
  int low_H = 0, low_S = 67, low_V = 78;
  int high_H = 0, high_S = 255, high_V = 255;
  Mat croppedImg;
  Mat image_HSV, image_threshold;
  // Convert from BGR to HSV colorspace
  bool result = false;
  //YELLOW
  if (color=="YELLOW") {
    int startX=210, startY=0, width=250, height=image.size().height;
    Mat ROI(image, Rect(startX,startY,width,height));
    ROI.copyTo(croppedImg);
    cvtColor(croppedImg, image_HSV, COLOR_BGR2HSV);
    low_H = 17;
    high_H = 30;
    inRange(image_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), image_threshold);
    int count = countNonZero(image_threshold);
    if (count > 5000)
      result = true;
    //cout << "YELLOW COUNT: " << count << endl;
  }
  //RED
  if (color=="RED") {
    int startX=425, startY=0, width=250, height=image.size().height;
    Mat ROI(image, Rect(startX,startY,width,height));
    ROI.copyTo(croppedImg);
    cvtColor(croppedImg, image_HSV, COLOR_BGR2HSV);
    low_H = 125;
    high_H = 180;
    inRange(image_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), image_threshold);
    int count = countNonZero(image_threshold);
    if (count > 4000)
      result = true;
    //cout << "RED COUNT: " << count << endl;
  }
  //BLUE
  if (color=="BLUE") {
    int startX=640, startY=0, width=250, height=image.size().height;
    Mat ROI(image, Rect(startX,startY,width,height));
    ROI.copyTo(croppedImg);
    cvtColor(croppedImg, image_HSV, COLOR_BGR2HSV);
    low_H = 82;
    high_H = 118;
    inRange(image_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), image_threshold);
    int count = countNonZero(image_threshold);
    if (count > 2000)
      result = true;
    //cout << "BLUE COUNT: " << count << endl;
  }

  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  namedWindow(window_capture_name);
  Mat cap, frame;
  for (size_t i = 0; i < 3; i++) {
    if (i==0) {
      cap = imread("/home/lasse/Desktop/imageRight.png", CV_LOAD_IMAGE_COLOR);
    }
    if (i==1) {
      cap = imread("/home/lasse/Desktop/imageWrong.png", CV_LOAD_IMAGE_COLOR);
    }
    if (i==2) {
      cap = imread("/home/lasse/Desktop/imageWrong2.png", CV_LOAD_IMAGE_COLOR);
    }
    frame = cap;
    if(frame.empty())
    {
        break;
    }
    bool result_y = check_image(frame, "YELLOW");
    bool result_r = check_image(frame, "RED");
    bool result_b = check_image(frame, "BLUE");

    cout << "YELLOW: " << result_y << endl;
    cout << "RED: " << result_r << endl;
    cout << "BLUE: " << result_b << endl;

    while(true){
      // Show the frames
      imshow(window_capture_name, frame);
      char key = (char) waitKey(30);
      if (key == 'q' || key == 27)
      {
          break;
      }
    }
  }
  //ros::spin();
  return 0;
}
