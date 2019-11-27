#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <chrono>
#include <numeric>

using namespace ur_rtde;

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