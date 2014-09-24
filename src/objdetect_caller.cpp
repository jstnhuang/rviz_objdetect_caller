#include "rviz_objdetect_caller/objdetect_caller.h"

#include <functional>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <ros/ros.h>

#include "pr2_interactive_object_detection/UserCommandAction.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pr2_interactive_object_detection::UserCommandAction;
using pr2_interactive_object_detection::UserCommandFeedbackConstPtr;
using pr2_interactive_object_detection::UserCommandGoal;
using pr2_interactive_object_detection::UserCommandResultConstPtr;

namespace rviz_objdetect_caller {
ObjDetectCaller::ObjDetectCaller():
    action_client_("object_detection_user_command", true) {
}

ObjDetectCaller::~ObjDetectCaller() {
}

void ObjDetectCaller::Start() {
  action_client_.waitForServer();
  SendSegmentationGoal();
}

void ObjDetectCaller::DoneCallback(const SimpleClientGoalState& state,
            const UserCommandResultConstPtr& result) {
  ROS_INFO("Segmentation finished in state [%s]", state.toString().c_str());
  SendSegmentationGoal();
}

void ObjDetectCaller::ActiveCallback() {
  // Do nothing.
}

void ObjDetectCaller::FeedbackCallback(const UserCommandFeedbackConstPtr& feedback) {
  // Do nothing.
}

void ObjDetectCaller::SendSegmentationGoal() {
  UserCommandGoal goal;
  goal.request = UserCommandGoal::SEGMENT;
  auto done = boost::bind(&ObjDetectCaller::DoneCallback, this, _1, _2);
  auto active = boost::bind(&ObjDetectCaller::ActiveCallback, this);
  auto feedback = boost::bind(&ObjDetectCaller::FeedbackCallback, this, _1);
  action_client_.sendGoal(goal, done, active, feedback);
}
}


