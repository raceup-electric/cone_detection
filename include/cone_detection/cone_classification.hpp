#ifndef CONE_CLASSIFICATION_HPP
#define CONE_CLASSIFICATION_HPP

#include <vector>  // For std::vector
#include <algorithm>  // For std::min

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

cone_detection::ConeType classifyCone(const pcl::PointCloud<pcl::PointXYZI>& cluster); 

float calculateHeight(const pcl::PointCloud<pcl::PointXYZI>& cluster);

float calculateBaseRadius(const pcl::PointCloud<pcl::PointXYZI>& cluster);

float calculateAverageIntensity(const pcl::PointCloud<pcl::PointXYZI>& cluster) ;

void normalizeIntensity(const pcl::PointCloud<pcl::PointXYZI>& input_cloud, pcl::PointCloud<pcl::PointXYZI>& output_cloud);

void calculateIntensityVector(const pcl::PointCloud<pcl::PointXYZI>& cluster, std::vector<float>& output_vector, int number_of_stripes);

#endif  