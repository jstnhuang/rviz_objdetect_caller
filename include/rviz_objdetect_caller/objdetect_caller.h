#ifndef RVIZ_OBJDETECT_CALLER
#define RVIZ_OBJDETECT_CALLER

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <ros/node_handle.h>
#include <string>

#include "pr2_interactive_object_detection/UserCommandAction.h"

namespace pr2iod = pr2_interactive_object_detection;

namespace rviz_objdetect_caller {
// Calls the object segmenter repeatedly.
//
// Usage:
//   ObjDetectCaller caller("object_detection_user_command");
//   caller.Start();
class ObjDetectCaller {
 public:
  ObjDetectCaller(const std::string& action_name);
  virtual ~ObjDetectCaller();
  void Start();

 private:
  bool SendSegmentationGoal(const ros::Duration& timeout);

  actionlib::SimpleActionClient<pr2iod::UserCommandAction> action_client_;
};
}

#endif
