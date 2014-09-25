#include "rviz_objdetect_caller/objdetect_caller.h"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <string>
#include <tabletop_object_detector/TabletopSegmentation.h>

#include "pr2_interactive_object_detection/UserCommandAction.h"
#include "rviz_objdetect_caller/Clusters.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pr2_interactive_object_detection::UserCommandAction;
using pr2_interactive_object_detection::UserCommandFeedbackConstPtr;
using pr2_interactive_object_detection::UserCommandGoal;
using pr2_interactive_object_detection::UserCommandResultConstPtr;
using tabletop_object_detector::TabletopSegmentation;
using tabletop_object_detector::TabletopSegmentationResponse;

namespace rviz_objdetect_caller {
ObjDetectCaller::ObjDetectCaller(
    const std::string& table_seg_name,
    const std::string& cluster_topic):
    node_handle_(""),
    segmentation_client_(
      node_handle_.serviceClient<TabletopSegmentation>(
        table_seg_name, false)),
    cluster_publisher_(
      node_handle_.advertise<Clusters>(cluster_topic, 10)) {
}

ObjDetectCaller::~ObjDetectCaller() {
}

void ObjDetectCaller::Start() {
  while(true) {
    Clusters clusters;
    bool success = DoSegmentation(&clusters);
    if (success) {
      cluster_publisher_.publish(clusters);
    }
    ros::spinOnce();
  }
}

bool ObjDetectCaller::DoSegmentation(Clusters* clusters) {
  TabletopSegmentation seg_service;
 
  if (!segmentation_client_.call(seg_service)) {
    ROS_WARN("Call to segmentation service failed.");
    return false;
  }
 
  if (seg_service.response.result != TabletopSegmentationResponse::SUCCESS) {
    ROS_WARN("Segmentation service returned error %d.",
      static_cast<int>(seg_service.response.result));
    return false;
  }

  for (const auto& cloud : seg_service.response.clusters) {
    clusters->point_clouds.push_back(cloud);
  }
  return true;
}
}


