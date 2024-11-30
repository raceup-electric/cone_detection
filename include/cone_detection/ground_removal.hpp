#ifndef GROUND_REMOVAL_HPP
#define GROUND_REMOVAL_HPP

#include "cone_detection/cone_type.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"  // For publishing markers
#include "visualization_msgs/msg/marker_array.hpp"  // For MarkerArray


#include "pcl_conversions/pcl_conversions.h"  // PCL-ROS conversions
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

void removeGroundRANSAC(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>& non_ground_cloud);


#endif  
