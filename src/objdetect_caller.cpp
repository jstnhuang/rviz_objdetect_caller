#include "rviz_objdetect_caller/objdetect_caller.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <string>

#include "pr2_interactive_object_detection/UserCommandAction.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pr2_interactive_object_detection::UserCommandAction;
using pr2_interactive_object_detection::UserCommandFeedbackConstPtr;
using pr2_interactive_object_detection::UserCommandGoal;
using pr2_interactive_object_detection::UserCommandResultConstPtr;

namespace rviz_objdetect_caller {
ObjDetectCaller::ObjDetectCaller(const std::string& action_name):
    action_client_(action_name, true) {
}

ObjDetectCaller::~ObjDetectCaller() {
}

void ObjDetectCaller::Start() {
  action_client_.waitForServer();
  while(true) {
    bool on_time = SendSegmentationGoal(ros::Duration(5.0));
    if (!on_time) {
      ROS_WARN("Segmentation took longer than 5 seconds.");
    }
    ros::spinOnce();
  }
}

bool ObjDetectCaller::SendSegmentationGoal(const ros::Duration& timeout) {
  ROS_INFO("Doing segmentation.");
  UserCommandGoal goal;
  goal.request = UserCommandGoal::SEGMENT;
  action_client_.sendGoal(goal);
  return action_client_.waitForResult(timeout);
}
}


