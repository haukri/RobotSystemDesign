#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>
#include <vector>

using namespace ur_rtde;

double theta = 0.39;

std::vector<double> offsetPose(std::vector<double> pose, double x_offset, double y_offset) {
  pose[0] += x_offset * cos(theta) - y_offset * sin(theta);
  pose[1] += x_offset * sin(theta) + y_offset * cos(theta);
  return pose;
}

int main(int argc, char *argv[])
{
  RTDEReceiveInterface rtde_receive("192.168.1.10");

    std::cout << "Actual q is: " << std::endl;
    for (const auto &d : rtde_receive.getActualQ())
      std::cout << d << ", ";
    std::cout << std::endl;

    std::cout << "Actual pose is: " << std::endl;
    for (const auto &d : rtde_receive.getTargetTCPPose())
      std::cout << d << ", ";
    std::cout << std::endl;



  return 0;
}