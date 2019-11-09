#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  namedWindow(window_capture_name);
  namedWindow(window_detection_name);
  Mat frame, frame_HSV, frame_threshold;
  Mat cap;
  int low_H = 0, low_S = 67, low_V = 78;
  int high_H = 0, high_S = 255, high_V = 255;
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
    for (size_t j = 0; j < 3; j++) {
      Mat croppedImg;
      //YELLOW
      if (j==0) {
        int startX=210, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
        low_H = 17;
        high_H = 30;
      }
      //RED
      if (j==1) {
        int startX=425, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
        low_H = 125;
        high_H = 180;
      }
      //BLUE
      if (j==2) {
        int startX=640, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
        low_H = 82;
        high_H = 118;
      }
      while(true){
        frame = croppedImg;
        if(frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        int count = countNonZero(frame_threshold);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }
      }
    }
  }
  //ros::spin();
  return 0;
}
