#include "cone_detection/cone_classification.hpp"

const float WHITE_STRIPE_INTENSITY_THRESHOLD = 2000.0f;  // Intensity threshold for white stripe detection

cone_detection::ConeType classifyCone(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
    // Calculate cone dimensions (height, radius)
    float height = calculateHeight(cluster);
    float base_radius = calculateBaseRadius(cluster);

    // First, classify by size (big orange)
    if (height > cone_detection::BIG_CONE_MIN_HEIGHT && height < cone_detection::BIG_CONE_MAX_HEIGHT) {
        return cone_detection::ConeType::BIG_ORANGE;  // Big orange cone based on size
    }

    // Divide the cluster into three sections: top, middle, and bottom
    pcl::PointCloud<pcl::PointXYZI> top_points, middle_points, bottom_points;

    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
      
    // Find the min and max z values in the cluster to determine the height range
    for (const auto& point : cluster.points) {
        if (point.z < z_min) z_min = point.z;
        if (point.z > z_max) z_max = point.z;
    }

    float z_middle_min = z_min + (z_max - z_min) / 3;
    float z_middle_max = z_min + 2 * (z_max - z_min) / 3;

    // Divide points into top, middle, and bottom based on z-coordinate
    for (const auto& point : cluster.points) {
        if (point.z < z_middle_min) {
            bottom_points.push_back(point);
        } else if (point.z > z_middle_max) {
            top_points.push_back(point);
        } else {
            middle_points.push_back(point);
        }
    }

    // Focus on the middle section for intensity analysis
    float avg_intensity_middle = calculateAverageIntensity(middle_points);

    // Classify the cone based on the middle section's intensity
    if (avg_intensity_middle > WHITE_STRIPE_INTENSITY_THRESHOLD) {
        return cone_detection::ConeType::BLUE;  // blue cone with a white stripe 
    } else {
        return cone_detection::ConeType::YELLOW;  // Yellow cone with a black stripe 
    }
}

    // Helper function to calculate the height of the cone
    float calculateHeight(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
        float min_z = std::numeric_limits<float>::max();
        float max_z = -std::numeric_limits<float>::max();

        for (const auto& point : cluster.points) {
            if (point.z < min_z) min_z = point.z;
            if (point.z > max_z) max_z = point.z;
        }

        return max_z - min_z;  // Height = difference between max and min z-values
    }

    // Helper function to calculate the base radius of the cone (distance from centroid to points)
    float calculateBaseRadius(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
        Eigen::Vector4f centroid(0, 0, 0, 0);
        for (const auto& point : cluster.points) {
            centroid[0] += point.x;
            centroid[1] += point.y;
            centroid[2] += point.z;
        }
        centroid /= cluster.points.size();

        // Calculate the average distance from the centroid to each point in the cluster
        float radius_sum = 0;
        for (const auto& point : cluster.points) {
            float distance = std::sqrt(std::pow(point.x - centroid[0], 2) + 
                                    std::pow(point.y - centroid[1], 2) + 
                                    std::pow(point.z - centroid[2], 2));
            radius_sum += distance;
        }

        return radius_sum / cluster.points.size();  // Average distance = radius
    }

    // Helper function to calculate the average intensity of a cluster
    float calculateAverageIntensity(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
        float intensity_sum = 0;
        for (const auto& point : cluster.points) {
            intensity_sum += point.intensity;
        }
        return intensity_sum / cluster.points.size();  // Average intensity
    }