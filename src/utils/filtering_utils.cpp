#include "cone_detection/filtering_utils.hpp"

// Define tunable constants for the rectangular region
const float X_MIN = -10.0;  // Minimum x-coordinate
const float X_MAX = 0.0;   // Maximum x-coordinate
const float Y_MIN = -2.5;  // Minimum y-coordinate
const float Y_MAX = 2.5;   // Maximum y-coordinate

// Car bounding box
const float CAR_X_MIN = -2.0; //front
const float CAR_X_MAX = 1.5; //back
const float CAR_Y_MIN = -0.75; //left
const float CAR_Y_MAX = 0.75; //right

void restrictedFOVFiltering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             pcl::PointCloud<pcl::PointXYZI>& output_cloud,
                             float max_height) {
    output_cloud.clear(); // Ensure the output cloud is empty before populating it

    // Iterate through the input cloud and apply all filters in a single loop
    for (const auto& point : input_cloud.points) {
        // Filter by Z (height), X, and Y
        if (point.z <= max_height &&
            point.x >= X_MIN && point.x <= X_MAX &&
            point.y >= Y_MIN && point.y <= Y_MAX) {
            
            // Exclude points inside the car's bounding box
            if (!(point.x >= CAR_X_MIN && point.x <= CAR_X_MAX &&
                  point.y >= CAR_Y_MIN && point.y <= CAR_Y_MAX)) {
                output_cloud.push_back(point);
            }
        }
    }
}
