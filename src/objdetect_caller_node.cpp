// ROS node that calls the object_detection_user_command action repeatedly.
//
// Usage:
//   roslaunch rviz_objdetect_caller objdetect_caller_node

#include <ros/ros.h>
#include "rviz_objdetect_caller/objdetect_caller.h"

using rviz_objdetect_caller::ObjDetectCaller;

int main (int argc, char **argv) {
  ros::init(argc, argv, "objdetect_caller");
  ObjDetectCaller caller("object_detection_user_command");
  caller.Start();
  ros::spin();

  return 0;
}
