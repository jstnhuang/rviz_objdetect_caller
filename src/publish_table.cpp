// Reads tabletop segmentation clusters and publishes the table.

#include "rviz_objdetect_caller/publish_table.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "object_manipulation_msgs/FindClusterBoundingBox.h"
#include "ros/ros.h"
#include "rviz_objdetect_caller/Clusters.h"
#include "tabletop_object_detector/Table.h"
#include "visualization_msgs/Marker.h"

#include <string>
#include <vector>

using geometry_msgs::Pose;
using geometry_msgs::Vector3;
using rviz_objdetect_caller::Clusters;
using std::string;
using std::vector;
using tabletop_object_detector::Table;
using visualization_msgs::Marker;

namespace {
static const string kFixedFrame = "/base_link";
static const string kNamespace = "table";
}

namespace rviz_objdetect_caller {
TablePublisher::TablePublisher(
    string clusters_topic,
    string table_topic):
  clusters_topic_(clusters_topic),
  node_handle_(),
  marker_pub_(node_handle_.advertise<Marker>("segmented_objects_table", 10)) {
}

TablePublisher::~TablePublisher() {
}

void TablePublisher::Start() {
  subscriber_ = node_handle_.subscribe(
    clusters_topic_,
    10,
    &TablePublisher::ClusterCallback,
    this);
}

void TablePublisher::MakeMarker(
    const Pose& pose,
    const Vector3& dimensions,
    Marker* marker) {
  marker->header.frame_id = kFixedFrame;
  marker->header.stamp = ros::Time::now();
  marker->ns = kNamespace;
  marker->id = 0;
  marker->action = Marker::ADD;
  marker->type = Marker::CUBE;
  marker->pose = pose;
  marker->scale = dimensions;
  marker->color.a = 1;
  marker->color.r = 0;
  marker->color.g = 0;
  marker->color.b = 1;
}

void TablePublisher::ClusterCallback(const Clusters& clusters) {
  Table table = clusters.table;
  Marker marker;
  Pose message_pose = table.pose.pose;
  Pose pose = message_pose;
  pose.position.x += table.x_min + (table.x_max - table.x_min) / 2;
  pose.position.y += table.y_min + (table.y_max + table.y_min) / 2;
  Vector3 dimensions;
  dimensions.x = table.x_max - table.x_min;
  dimensions.y = table.y_max - table.y_min;
  dimensions.z = 0.05;
  MakeMarker(
    pose,
    dimensions,
    &marker);
  marker_pub_.publish(marker);
}
}  // namespace rviz_objdetect_caller
