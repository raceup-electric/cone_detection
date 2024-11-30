#include "cone_detection/filtering_utils.hpp"

void filterAboveHeight(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                           pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                           float max_height) {
        for (const auto& point : input_cloud.points) {
            if (point.z <= max_height) {
                output_cloud.push_back(point);
            }
        }
    }

void filterDistantPoints(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                             float max_distance) {
        for (const auto& point : input_cloud.points) {
            if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) <= max_distance) {
                output_cloud.push_back(point);
            }
        }
    }

void coordinateBasedFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                        pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                        float max_height, float max_distance) {
    for (const auto& point : input_cloud.points) {
        if ((point.z <= max_height) && (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) <= max_distance)) {
            output_cloud.push_back(point);
        }
    }
}

// Define tunable constants for the rectangular region
const float X_MIN = -10.0f;  // Minimum x-coordinate
const float X_MAX = 1.0f;   // Maximum x-coordinate
const float Y_MIN = -3.5f;  // Minimum y-coordinate
const float Y_MAX = 3.5f;   // Maximum y-coordinate

void restrictedFOVFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                             float max_height) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input_cloud.makeShared());

    // Filter by Z (height)
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<float>::max(), max_height);
    pcl::PointCloud<pcl::PointXYZI> temp_cloud_z;
    pass.filter(temp_cloud_z);

    // Filter by X (horizontal distance)
    pass.setInputCloud(temp_cloud_z.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(X_MIN, X_MAX);
    pcl::PointCloud<pcl::PointXYZI> temp_cloud_x;
    pass.filter(temp_cloud_x);

    // Filter by Y
    pass.setInputCloud(temp_cloud_x.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_MIN, Y_MAX);
    pass.filter(output_cloud);
}

