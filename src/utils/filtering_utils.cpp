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