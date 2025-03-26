#include "cone_detection/filtering_utils.hpp"

void restrictedFOVFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                             MissionConfig config) {
    output_cloud.clear(); // Ensure the output cloud is empty before populating it

    // Iterate through the input cloud and apply all filters in a single loop
    for (const auto& point : input_cloud.points) {
        // Filter by Z (height), X, and Y
        if (point.z <= config.height_filter &&
            point.x >= config.fov_x_min && point.x <= config.fov_x_max &&
            point.y >= config.fov_y_min && point.y <= config.fov_y_max) {
            
            // Exclude points inside the car's bounding box
            if (!(point.x >= config.car_x_min && point.x <= config.car_x_max &&
                  point.y >= config.car_y_min && point.y <= config.car_y_max)) {
                output_cloud.push_back(point);
            }
        }
    }
}
