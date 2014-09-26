// Reads tabletop segmentation clusters and publishes bounding boxes.

#include "rviz_objdetect_caller/publish_boxes.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>

#include "object_manipulation_msgs/FindClusterBoundingBox.h"
#include "rviz_objdetect_caller/Clusters.h"
#include "visualization_msgs/Marker.h"

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using object_manipulation_msgs::FindClusterBoundingBox;
using object_manipulation_msgs::FindClusterBoundingBoxResponse;
using rviz_objdetect_caller::Clusters;
using sensor_msgs::ChannelFloat32;
using sensor_msgs::PointCloud;
using std::string;
using std::vector;
using visualization_msgs::Marker;

namespace {
static const string kFixedFrame = "/base_link";
static const string kNamespace = "bounding_boxes";
}

namespace rviz_objdetect_caller {
BoxPublisher::BoxPublisher(
    string clusters_topic,
    string bounding_box_service,
    string box_topic):
  clusters_topic_(clusters_topic),
  node_handle_(),
  marker_pub_(node_handle_.advertise<Marker>("segmented_objects_boxes", 10)),
  bounding_box_client_(
    node_handle_.serviceClient<FindClusterBoundingBox>(
      bounding_box_service, false)),
  prev_number_clusters_(0) {
}

BoxPublisher::~BoxPublisher() {
}

void BoxPublisher::Start() {
  subscriber_ = node_handle_.subscribe(
    clusters_topic_,
    10,
    &BoxPublisher::ClusterCallback,
    this);
}

void BoxPublisher::ClearMarkers() {
  for (int i = 0; i < prev_number_clusters_; ++i) {
    Marker marker;
    marker.header.frame_id = kFixedFrame;
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    marker.id = i;
    marker.action = Marker::DELETE;
    marker_pub_.publish(marker);
  }
}

void BoxPublisher::MakeMarker(
    const int i,
    const Pose& pose,
    const Vector3& dimensions,
    Marker* marker) {
  marker->header.frame_id = kFixedFrame;
  marker->header.stamp = ros::Time::now();
  marker->ns = kNamespace;
  marker->id = i;
  marker->action = Marker::ADD;
  marker->type = Marker::CUBE;
  marker->pose = pose;
  marker->scale = dimensions;
  marker->color.a = 1;
  marker->color.r = 1;
  marker->color.g = 0;
  marker->color.b = 1;
}

void BoxPublisher::ClusterCallback(const Clusters& clusters) {
  ClearMarkers();
  prev_number_clusters_ = clusters.point_clouds.size();

  // The bounding box service call may take time, so cache the responses.
  // That way, when we call MakeMarker, the header is not out of date.
  vector<FindClusterBoundingBoxResponse> responses;
  for (unsigned long i=0; i<clusters.point_clouds.size(); ++i) {
    const auto& point_cloud = clusters.point_clouds[i];
    FindClusterBoundingBox service;
    service.request.cluster = point_cloud;
    if (bounding_box_client_.call(service)
      && (service.response.error_code
        == FindClusterBoundingBoxResponse::SUCCESS)) {
      responses.push_back(service.response);
      } else {
      ROS_WARN("Failed to get bounding box.");
    }
  }

  for (unsigned long i=0; i<responses.size(); ++i) {
    auto response = responses[i];
    Marker marker;
    MakeMarker(
      static_cast<int>(i),
      response.pose.pose,
      response.box_dims,
      &marker);
    marker_pub_.publish(marker);
  }
}
}  // namespace rviz_objdetect_caller
