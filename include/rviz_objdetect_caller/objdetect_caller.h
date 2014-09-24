#ifndef RVIZ_OBJDETECT_CALLER
#define RVIZ_OBJDETECT_CALLER

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include "pr2_interactive_object_detection/UserCommandAction.h"

namespace pr2iod = pr2_interactive_object_detection;

namespace rviz_objdetect_caller {
class ObjDetectCaller {
 public:
  ObjDetectCaller();
  ~ObjDetectCaller();
  void Start();

 private:
  void DoneCallback(const actionlib::SimpleClientGoalState& state,
            const pr2iod::UserCommandResultConstPtr& result);
  void ActiveCallback();
  void FeedbackCallback(const pr2iod::UserCommandFeedbackConstPtr& feedback);
  void SendSegmentationGoal();

  actionlib::SimpleActionClient<pr2iod::UserCommandAction> action_client_;
};
}

#endif
