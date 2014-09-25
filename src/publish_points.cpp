// Reads tabletop segmentation clusters and publishes the point cloud.

#include "rviz_objdetect_caller/publish_points.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>

#include "rviz_objdetect_caller/Clusters.h"

using rviz_objdetect_caller::Clusters;
using sensor_msgs::ChannelFloat32;
using sensor_msgs::PointCloud;
using std::string;
using std::vector;

namespace rviz_objdetect_caller {
PointPublisher::PointPublisher(string clusters_topic, string cloud_topic):
  clusters_topic_(clusters_topic),
  node_handle_(),
  publisher_(node_handle_.advertise<PointCloud>(cloud_topic, 10)) {
}

PointPublisher::~PointPublisher() {
}

void PointPublisher::Start() {
  subscriber_ = node_handle_.subscribe(
    clusters_topic_,
    10,
    &PointPublisher::ClusterCallback,
    this);
}

void PointPublisher::ConcatenatePointClouds(
    const vector<PointCloud>& point_clouds,
    PointCloud* output) {
  if (point_clouds.size() > 0) {
    output->header = point_clouds[0].header;
  }
  for (const PointCloud& cloud : point_clouds) {
    output->points.insert(
      output->points.end(), cloud.points.begin(), cloud.points.end());
    for (unsigned long channel=0; channel<cloud.channels.size(); ++channel) {
      if (channel < output->channels.size()) {
        // 2nd pass and onwards: append to existing channels.
        auto* output_values = &(output->channels[channel].values);
        const auto& cloud_values = cloud.channels[channel].values;
        output_values->insert(output_values->end(),
            cloud_values.begin(), cloud_values.end());
      } else {
        // First pass: copy and insert channels.
        output->channels.push_back(cloud.channels[channel]);
      }
    }
  }
}

void PointPublisher::ClusterCallback(const Clusters& clusters) {
  PointCloud concatenated;
  ConcatenatePointClouds(clusters.point_clouds, &concatenated);
  publisher_.publish(concatenated); 
}
}  // namespace rviz_objdetect_caller
