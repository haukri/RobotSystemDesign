#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <iostream>
#include <std_msgs/Bool.h>

using namespace cv;
using namespace std;

vector<vector<Point>> get_contours(Mat, int, int, int, int, int, int );
bool check_order(vector<vector<Point>>, vector<vector<Point>>, vector<vector<Point>>, vector<int>);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    vector<int> order = {2, 1, 1};

    vector<vector<Point>> contours_blue = get_contours(image, 52, 118, 72, 255, 0, 255);
    vector<vector<Point>> contours_red = get_contours(image, 0, 15, 72, 255, 0, 255);
    vector<vector<Point>> contours_yellow = get_contours(image, 16, 36, 72, 255, 0, 255);

    bool result = check_order(contours_blue, contours_red, contours_yellow, order);

    if(result)
      cout << "Right stuff" << endl;
    else
      cout << "Wrong stuff" << endl;

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("vision/seg", 1);
    std_msgs::Bool pub_msg;
    pub_msg.data = result;

    chatter_pub.publish(pub_msg);
    ros::spinOnce();

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}

vector<vector<Point>> get_contours(Mat image, int H_min, int H_max, int S_min, int S_max, int V_min, int V_max) {
  Mat image_hsv, image_seg;
  cvtColor(image, image_hsv, COLOR_BGR2HSV);
  // Detect the object based on HSV Range Values
  inRange(image_hsv, Scalar(H_min, S_min, V_min), Scalar(H_min, S_min, V_min), image_seg);

  Mat canny;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  Canny( image_seg, canny, 100, 200 );
  findContours(canny, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE );
  return contours;
}

bool check_order(vector<vector<Point>> blue, vector<vector<Point>> red, vector<vector<Point>> yellow, vector<int> order) {
  bool result = true;
  //Count Blue
  if (order[0] == blue.size()) {
    //Check if they are small
    for( size_t i = 0; i< blue.size(); i++ ) {
      if (contourArea(blue[i])>30000)
        result = false;
    }
  }
  else
    result = false;

  //Count Red
  if (order[1] == red.size()) {
    //Check if they are normal
    for( size_t i = 0; i< red.size(); i++ ) {
      if (contourArea(red[i])>60000)
        result = false;
    }
  }
  else
    result = false;

  //Count Yellow
  if (order[2] == yellow.size()) {
    //Check if they are normal
    for( size_t i = 0; i< yellow.size(); i++ ) {
      if (contourArea(yellow[i])>60000)
        result = false;
    }
  }
  else
    result = false;

  return result
}
