// Reads tabletop segmentation clusters and publishes bounding boxes.

#include <ros/ros.h>
#include <vector>

#include "rviz_objdetect_caller/publish_boxes.h"

using rviz_objdetect_caller::Clusters;
using rviz_objdetect_caller::BoxPublisher;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_boxes_node");
  BoxPublisher publisher(
    "table_clusters",
    "find_cluster_bounding_box",
    "segmented_objects_boxes");
  publisher.Start();
  ros::spin();
  return 0;
}
