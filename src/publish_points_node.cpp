// Reads tabletop segmentation clusters and publishes the point cloud.

#include <ros/ros.h>

#include "rviz_objdetect_caller/publish_points.h"

using rviz_objdetect_caller::PointPublisher;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_points_node");
  PointPublisher publisher("table_clusters", "segmented_objects_points");
  publisher.Start();
  ros::spin();
  return 0;
}
