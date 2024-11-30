#include "cone_detection/ground_removal.hpp"

const float DISTANCE_THRESHOLD = 0.05;       // RANSAC distance threshold for ground plane
const int NUM_ITERATIONS = 10;             // RANSAC number of iterations

void removeGroundRANSAC(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>& non_ground_cloud) {

    // Apply voxel grid downsampling to reduce the number of points
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    pcl::PointCloud<pcl::PointXYZI> downsampled_cloud;

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
        non_ground_cloud = input_cloud;  // No segmentation, return all points
        return;
    }

    // Extract non-ground points in a single step
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(input_cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);  // Extract non-ground points directly
    extract.filter(non_ground_cloud);

} 



