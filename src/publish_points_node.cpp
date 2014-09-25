// Reads tabletop segmentation clusters and publishes the point cloud.

#include <functional>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>

#include "rviz_objdetect_caller/Clusters.h"
#include "rviz_objdetect_caller/publish_points.h"

using rviz_objdetect_caller::Clusters;
using rviz_objdetect_caller::PointPublisher;
using sensor_msgs::ChannelFloat32;
using sensor_msgs::PointCloud;
using std::vector;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_points_node");
  PointPublisher publisher("table_clusters", "segmented_objects_points");
  publisher.Start();
  ros::spin();
  return 0;
}
