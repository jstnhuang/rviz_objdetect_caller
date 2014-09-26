// Reads tabletop segmentation clusters and publishes the table.

#include <ros/ros.h>
#include <vector>

#include "rviz_objdetect_caller/publish_table.h"

using rviz_objdetect_caller::Clusters;
using rviz_objdetect_caller::TablePublisher;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_table_node");
  TablePublisher publisher(
    "table_clusters",
    "segmented_objects_table");
  publisher.Start();
  ros::spin();
  return 0;
}
