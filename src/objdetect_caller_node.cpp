#include <ros/ros.h>
#include "rviz_objdetect_caller/objdetect_caller.h"

using rviz_objdetect_caller::ObjDetectCaller;

int main (int argc, char **argv) {
  ros::init(argc, argv, "objdetect_caller");
  ObjDetectCaller caller;
  caller.Start();

  return 0;
}
