#include "cone_detection/ground_removal.hpp"

const float HEIGHT_THRESHOLD = 0.5;          // Maximum height for ground points
const float DISTANCE_THRESHOLD = 0.05;       // RANSAC distance threshold for ground plane
const int NUM_ITERATIONS = 1000;             // RANSAC number of iterations


void removeGroundRANSAC(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                            pcl::PointCloud<pcl::PointXYZI>& non_ground_cloud,
                            pcl::PointCloud<pcl::PointXYZI>& ground_cloud) {
        // RANSAC-based ground plane segmentation
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(DISTANCE_THRESHOLD);
        seg.setMaxIterations(NUM_ITERATIONS);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        seg.setInputCloud(input_cloud.makeShared());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            //RCLCPP_WARN(this->get_logger(), "No ground plane found!");
            non_ground_cloud = input_cloud;
            return;
        }

        // Extract ground points
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(input_cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(ground_cloud);

        // Filter out ground points above HEIGHT_THRESHOLD
        pcl::PointCloud<pcl::PointXYZI> filtered_ground_cloud;
        for (const auto& point : ground_cloud.points) {
            if (point.z <= HEIGHT_THRESHOLD) {
                filtered_ground_cloud.push_back(point);
            }
        }
        ground_cloud = filtered_ground_cloud;

        // Extract non-ground points
        extract.setNegative(true);
        extract.filter(non_ground_cloud);
    }