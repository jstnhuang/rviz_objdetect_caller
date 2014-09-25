#ifndef RVIZ_OBJDETECT_CALLER
#define RVIZ_OBJDETECT_CALLER

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <ros/node_handle.h>
#include <string>
#include <tabletop_object_detector/TabletopSegmentation.h>

#include "pr2_interactive_object_detection/UserCommandAction.h"
#include "rviz_objdetect_caller/Clusters.h"

namespace pr2iod = pr2_interactive_object_detection;
namespace tod = tabletop_object_detector;

namespace rviz_objdetect_caller {
// Calls the object segmenter repeatedly and publishes the clusters.
//
// Usage:
//   ObjDetectCaller caller("tabletop_segmentation", "table_clusters");
//   caller.Start();
class ObjDetectCaller {
 public:
  ObjDetectCaller(
    const std::string& table_seg_name,
    const std::string& cluster_topic);
  virtual ~ObjDetectCaller();
  void Start();

 private:
  bool DoSegmentation(Clusters* clusters);
  ros::NodeHandle node_handle_;
  ros::ServiceClient segmentation_client_;
  ros::Publisher cluster_publisher_;
};
}

#endif
