#ifndef OBJDETECT_CALLER_PUBLISH_POINTS
#define OBJDETECT_CALLER_PUBLISH_POINTS

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>

#include "rviz_objdetect_caller/Clusters.h"
#include "visualization_msgs/Marker.h"

namespace rviz_objdetect_caller {
class BoxPublisher {
 public:
  BoxPublisher(
    std::string clusters_topic,
    std::string bounding_box_service,
    std::string box_topic);
  virtual ~BoxPublisher();
  void Start();

 private:
  void ClearOldMarkers(int num_current_clusters);
  void MakeMarker(
    const int i,
    const geometry_msgs::Pose& pose,
    const geometry_msgs::Vector3& dimensions,
    visualization_msgs::Marker* marker);
  void ClusterCallback(const Clusters& clusters);
  std::string clusters_topic_;
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  ros::Publisher marker_pub_;
  ros::ServiceClient bounding_box_client_;
  int prev_number_clusters_;
};
}  // namespace rviz_objdetect_caller

#endif
