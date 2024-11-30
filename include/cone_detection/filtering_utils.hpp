#ifndef FILTERING_UTILS_HPP
#define FILTERING_UTILS_HPP

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
#include <pcl/kdtree/kdtree.h>
#include <Eigen/Dense>  // For centroid calculation
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/passthrough.h>

void filterAboveHeight(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                           pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                           float max_height);

void filterDistantPoints(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                            float max_distance); 

void coordinateBasedFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                           pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                           float max_height, float max_distance);

void restrictedFOVFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                            float max_height);
                    
#endif  
