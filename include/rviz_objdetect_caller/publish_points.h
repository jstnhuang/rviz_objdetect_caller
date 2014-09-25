#ifndef OBJDETECT_CALLER_PUBLISH_POINTS
#define OBJDETECT_CALLER_PUBLISH_POINTS

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>

#include "rviz_objdetect_caller/Clusters.h"

namespace rviz_objdetect_caller {
class PointPublisher {
 public:
  PointPublisher(std::string clusters_topic, std::string cloud_topic);
  virtual ~PointPublisher();
  void Start();

 private:
  void ConcatenatePointClouds(
    const std::vector<sensor_msgs::PointCloud>& point_clouds,
    sensor_msgs::PointCloud* output);
  void ClusterCallback(const Clusters& clusters);
  std::string clusters_topic_;
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
}  // namespace rviz_objdetect_caller

#endif
