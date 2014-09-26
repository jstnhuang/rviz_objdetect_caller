#ifndef OBJDETECT_CALLER_PUBLISH_POINTS
#define OBJDETECT_CALLER_PUBLISH_POINTS

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <string>
#include <vector>

#include "rviz_objdetect_caller/Clusters.h"
#include "visualization_msgs/Marker.h"

namespace rviz_objdetect_caller {
class TablePublisher {
 public:
  TablePublisher(
    std::string clusters_topic,
    std::string table_topic);
  virtual ~TablePublisher();
  void Start();

 private:
  void MakeMarker(
    const geometry_msgs::Pose& pose,
    const geometry_msgs::Vector3& dimensions,
    visualization_msgs::Marker* marker);
  void ClusterCallback(const Clusters& clusters);
  std::string clusters_topic_;
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher marker_pub_;
};
}  // namespace rviz_objdetect_caller

#endif
