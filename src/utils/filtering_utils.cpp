#include "cone_detection/filtering_utils.hpp"

// Define tunable constants for the rectangular region
const float X_MIN = -20.0;  // Minimum x-coordinate
const float X_MAX = 0.0;   // Maximum x-coordinate
const float Y_MIN = -3.5;  // Minimum y-coordinate
const float Y_MAX = 3.5;   // Maximum y-coordinate

// Car bounding box
const float CAR_X_MIN = -2.0; //front
const float CAR_X_MAX = 1.5; //back
const float CAR_Y_MIN = -0.75; //left
const float CAR_Y_MAX = 0.75; //right

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

    // Filter by Y (lateral distance)
    pass.setInputCloud(temp_cloud_x.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(Y_MIN, Y_MAX);
    pcl::PointCloud<pcl::PointXYZI> temp_cloud_y;
    pass.filter(temp_cloud_y);

    // Keep points inside the first rectangle but exclude points inside the car's bounding box
    output_cloud.clear(); // Ensure the output cloud is empty before populating it
    for (const auto& point : temp_cloud_y.points) {
        // Check if the point is outside the car's bounding box
        if (!(point.x >= CAR_X_MIN && point.x <= CAR_X_MAX &&
              point.y >= CAR_Y_MIN && point.y <= CAR_Y_MAX)) {
            output_cloud.push_back(point);
        }
    }
}

