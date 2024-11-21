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

// Tunable parameters
const float DISTANCE_THRESHOLD = 0.05;       // RANSAC distance threshold for ground plane
const int NUM_ITERATIONS = 1000;             // RANSAC number of iterations
const float HEIGHT_THRESHOLD = 0.5;          // Maximum height for ground points
const float LIDAR_HEIGHT = 0.6;
const float HEIGHT_FILTER = 0.8;
const float MAX_HEIGHT_THRESHOLD = HEIGHT_FILTER - LIDAR_HEIGHT;  // Maximum allowed height for all points (non-ground points)
const float DISTANCE_RADIUS_THRESHOLD = 10.0;       // Distance threshold for filtering

// DBSCAN parameters
const float EPS = 0.5;                       // Cluster tolerance (distance)
const int MIN_POINTS = 10;                   // Minimum points per cluster

// Cone size constraints
const float SMALL_CONE_MIN_HEIGHT = 0.1;
const float SMALL_CONE_MAX_HEIGHT = 0.3;
const float SMALL_CONE_BASE_RADIUS = 0.20;

const float BIG_CONE_MIN_HEIGHT = 0.3;
const float BIG_CONE_MAX_HEIGHT = 0.6;
const float BIG_CONE_BASE_RADIUS = 0.26;


const int MIN_CLUSTER_SIZE = 20;  // Minimum number of points for a cluster to be considered valid
const int MAX_CLUSTER_SIZE = 200; // Maximum number of points for a cluster to be considered valid


const float STRIPE_INTENSITY_THRESHOLD = 0.3f;  // Minimum intensity variation to detect stripes
const float WHITE_STRIPE_INTENSITY_THRESHOLD = 2000.0f;  // Intensity threshold for white stripe detection (blue and orange cones)
const float BLACK_STRIPE_INTENSITY_THRESHOLD = 50.0f; 

enum ConeType {
    BIG_ORANGE,
    BLUE,
    YELLOW,
    ORANGE,
    UNKNOWN
};


class ConeDetectionNode : public rclcpp::Node {
public:
    ConeDetectionNode() : Node("cone_detection_node") {
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", 10,
            std::bind(&ConeDetectionNode::pointCloudCallback, this, std::placeholders::_1));

        cone_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/detected_cones", 10);

                marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_markers", 10);
    }

private:

   void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert PointCloud2 to PCL PointCloud<PointXYZI> for processing
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // Filter out points that are too high
    pcl::PointCloud<pcl::PointXYZI> height_filtered_cloud;
    filterAboveHeight(pcl_cloud, height_filtered_cloud, MAX_HEIGHT_THRESHOLD);

    // Separate ground and non-ground points using RANSAC
    pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
    pcl::PointCloud<pcl::PointXYZI> ground_cloud;
    removeGroundRANSAC(height_filtered_cloud, ground_removed_cloud, ground_cloud);

    // Filter out distant points
    pcl::PointCloud<pcl::PointXYZI> distance_filtered_cloud;
    filterDistantPoints(ground_removed_cloud, distance_filtered_cloud, DISTANCE_RADIUS_THRESHOLD);

    // Cluster the remaining points
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cone_clusters;
    performDBSCANClustering(distance_filtered_cloud, cone_clusters);

    // Classify clusters and store classified cones
    std::vector<pcl::PointCloud<pcl::PointXYZI>> classified_cones;
    for (auto& cluster : cone_clusters) {
        if (cluster.points.size() >= MIN_POINTS && cluster.points.size() <= 25000) {
            // Only keep clusters within point count range
            ConeType cone_type = classifyCone(cluster);
            if (cone_type != UNKNOWN) {
                classified_cones.push_back(cluster);  // Only store recognized cones
            }
        }
    }

    // Print intensity values of the first cluster that fits the criteria
    if (!classified_cones.empty()) {
        const pcl::PointCloud<pcl::PointXYZI>& first_cluster = classified_cones[0];
        RCLCPP_INFO(this->get_logger(), "Intensity values for the first detected cluster:");
        for (const auto& point : first_cluster.points) {
            RCLCPP_INFO(this->get_logger(), "Intensity: %f", point.intensity);
        }
    }

    // Publish detected cone clusters
    publishConePointCloud(classified_cones, msg->header);

    // Publish markers for each classified cone
    publishConeMarkers(classified_cones, msg->header);
}



    void publishConePointCloud(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters,
                               const std_msgs::msg::Header& header) {
        pcl::PointCloud<pcl::PointXYZI> cones_cloud;
        for (const auto& cluster : cone_clusters) {
            cones_cloud += cluster;
        }
        sensor_msgs::msg::PointCloud2 cones_msg;
        pcl::toROSMsg(cones_cloud, cones_msg);
        cones_msg.header = header;
        cone_publisher_->publish(cones_msg);
    }

   void publishConeMarkers(const std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters,
                        const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (const auto& cluster : cone_clusters) {
            if (cluster.points.empty()) continue;

            Eigen::Vector4f centroid(0, 0, 0, 0);
            for (const auto& point : cluster.points) {
                centroid[0] += point.x;
                centroid[1] += point.y;
                centroid[2] += point.z;
            }
            centroid /= cluster.points.size();

            ConeType cone_type = classifyCone(cluster);
            
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "cone_markers";
            marker.id = marker_array.markers.size();  // Unique ID
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            
            // Set dimensions and colors based on cone type
            if (cone_type == BIG_ORANGE) {
                marker.scale.x = BIG_CONE_BASE_RADIUS * 2;
                marker.scale.y = BIG_CONE_BASE_RADIUS * 2;
                marker.scale.z = BIG_CONE_MAX_HEIGHT;
                marker.color.r = 1.0f;  // Orange
                marker.color.g = 0.55f;
                marker.color.b = 0.0f;
            } else if (cone_type == BLUE) {
                marker.scale.x = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.y = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.z = SMALL_CONE_MAX_HEIGHT;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;  // Blue
            } else if (cone_type == YELLOW) {
                marker.scale.x = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.y = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.z = SMALL_CONE_MAX_HEIGHT;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;  // Yellow
            } else if (cone_type == ORANGE) {
                marker.scale.x = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.y = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.z = SMALL_CONE_MAX_HEIGHT;
                marker.color.r = 1.0f;  // Orange
                marker.color.g = 0.55f;
                marker.color.b = 0.0f;
            } else {
                marker.scale.x = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.y = SMALL_CONE_BASE_RADIUS * 2;
                marker.scale.z = SMALL_CONE_MAX_HEIGHT;
                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;  // Unknown color (grey)
            }
            marker.color.a = 1.0; // Full opacity

            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);
    }


    ConeType classifyCone(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
        // Calculate cone dimensions (height, radius)
        float height = calculateHeight(cluster);
        float base_radius = calculateBaseRadius(cluster);

        // First, classify by size (big orange)
        if (height > BIG_CONE_MIN_HEIGHT && height < BIG_CONE_MAX_HEIGHT &&
            base_radius > BIG_CONE_BASE_RADIUS) {
            return BIG_ORANGE;  // Big orange cone based on size
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
            return BLUE;  // blue cone with a white stripe 
        } else {
            return YELLOW;  // Yellow cone with a black stripe 
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

    // Helper function to calculate the intensity variation in a cluster
    float calculateIntensityVariation(const pcl::PointCloud<pcl::PointXYZI>& cluster) {
        float intensity_variation = 0;
        float avg_intensity = calculateAverageIntensity(cluster);
        for (const auto& point : cluster.points) {
            intensity_variation += std::pow(point.intensity - avg_intensity, 2);
        }
        return std::sqrt(intensity_variation / cluster.points.size());  // Standard deviation of intensity
    }

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
            RCLCPP_WARN(this->get_logger(), "No ground plane found!");
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

   void performDBSCANClustering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters) {
        // Parameters for the number of points in a cluster

        // Build a KD-tree for clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(input_cloud.makeShared());

        // Perform DBSCAN-based clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(EPS);
        ec.setMinClusterSize(MIN_POINTS);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud.makeShared());
        ec.extract(cluster_indices);

        // Store each cluster in a separate PointCloud
        for (const auto& indices : cluster_indices) {
            // Check if the cluster has the desired number of points
            if (indices.indices.size() >= MIN_CLUSTER_SIZE && indices.indices.size() <= MAX_CLUSTER_SIZE) {
                pcl::PointCloud<pcl::PointXYZI> cluster;
                for (const auto& index : indices.indices) {
                    cluster.points.push_back(input_cloud.points[index]);
                }
                cone_clusters.push_back(cluster);
            }
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
