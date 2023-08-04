#include "ArxDriver.h"
#include "ros/ros.h"

int main(int argc, char** argv){
  ros::init(argc,argv,"driverTest");
  ros::NodeHandle nh; //without this ,can't enter deconstructor
  ArxDriver driver(500);
  driver.Init();

  while(ros::ok()){
    driver.Update();
  }
  return 0;
}