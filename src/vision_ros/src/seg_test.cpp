#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>

using namespace cv;

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);

  namedWindow(window_capture_name);
  namedWindow(window_detection_name);
  // Trackbars to set thresholds for HSV values
  createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
  createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
  createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
  createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
  createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
  createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

  Mat frame, frame_HSV, frame_threshold;
  Mat cap;
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
      if (j==0) {
        int startX=210, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
      }
      if (j==1) {
        int startX=425, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
      }
      if (j==2) {
        int startX=640, startY=0, width=250, height=cap.size().height;
        Mat ROI(cap, Rect(startX,startY,width,height));
        ROI.copyTo(croppedImg);
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
