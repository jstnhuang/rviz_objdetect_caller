rviz_objdetect_caller
=====================

"rviz" is a bit of a misnomer now. This package provides three nodes:

1. *objdetect_caller*: Calls the PR2 interactive object backend's segmenter on repeat, and publishes the point clouds to a topic.
2. *publish_points*: Concatenates the point clouds for all the segmented objects from objdetect_caller, and publishes the concatenated point cloud.
3. *publish_boxes*: Finds the bounding box around the segmented objects from objdetect_caller, and publishes the boxes as markers.
