#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  cv::VideoCapture inputVideo;
  inputVideo.open(0);
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
  cv::Point2f groundTruth;
  for (size_t i = 1; i < 9; i++) {
    while(true) {
      cv::Mat image, imageCopy;
      image = imread("/home/lasse/Desktop/marker" + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);
      image.copyTo(imageCopy);
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f> > corners;
      cv::aruco::detectMarkers(image, dictionary, corners, ids);
      if(i==1) {
        groundTruth = corners[0][0];
      }
      cout << corners[0][0] << endl;
      //cout << corners[0][0] - groundTruth << endl;
      // if at least one marker detected
      if (ids.size() > 0) {
          cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
      }
      cv::imshow("out", imageCopy);
      char key = (char) cv::waitKey(30);
      if (key == 27)
          break;
      }
  }
  return 0;
}
