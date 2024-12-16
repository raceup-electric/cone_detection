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



#include "cone_detection/cone_type.hpp"
#include "cone_detection/cone_classification.hpp"
#include "cone_detection/filtering_utils.hpp"
#include "cone_detection/ground_removal.hpp"
#include "cone_detection/cone_clustering.hpp"

#include <fstream>
#include <time.h>

#include "eufs_msgs/msg/cone_array_with_covariance.hpp"

//CONSTANTS

// LiDAR positioning with respect to the ground
const float LIDAR_HEIGHT = 0.97;

// Height limit for the points in the pointcloud
const float HEIGHT_FILTER = 0.7;

// Maximum allowed height for all points (non-ground points)
const float MAX_HEIGHT_THRESHOLD = HEIGHT_FILTER - LIDAR_HEIGHT;  

// Cluster size thresholds
const int MIN_POINTS = 10;                   
const int MAX_POINTS = 500;

// Cone size constraints for RVIZ2 Markers
const float MARKER_SMALL_CONE_HEIGHT = cone_detection::SMALL_CONE_HEIGHT;
const float MARKER_SMALL_CONE_RADIUS = cone_detection::SMALL_CONE_BASE_RADIUS;
const float MARKER_BIG_CONE_HEIGHT = cone_detection::BIG_CONE_HEIGHT;
const float MARKER_BIG_CONE_RADIUS = cone_detection::BIG_CONE_BASE_RADIUS;

class ConeDetectionNode : public rclcpp::Node {
public:
    ConeDetectionNode() : Node("cone_detection_node") {
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ouster/points", 10,
                std::bind(&ConeDetectionNode::pointCloudCallback, this, std::placeholders::_1));

        cone_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_cones", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_markers", 10);

        restricted_fov_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/restricted_fov", 10);
        
        cone_pub = this->create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("/cone_pose", 1);
    }

private:

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Calculate the callback duration
        rclcpp::Time first_time = rclcpp::Clock(RCL_STEADY_TIME).now();

        // Convert PointCloud2 to PCL PointCloud<PointXYZI> for processing
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Filter out points based on their coordinates
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
        restrictedFOVFiltering(pcl_cloud, filtered_cloud, MAX_HEIGHT_THRESHOLD);


        //This is an optional publisher to see the field of view that is kept
        //TODO: Comment this if it is not useful
        sensor_msgs::msg::PointCloud2 restricted_fov_msg;
        pcl::toROSMsg(filtered_cloud, restricted_fov_msg);
        restricted_fov_msg.header = msg->header;
        restricted_fov_publisher_->publish(restricted_fov_msg);

        // Separate ground and non-ground points using RANSAC
        pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
        removeGroundRANSAC(filtered_cloud, ground_removed_cloud);

        // Cluster the remaining points
        std::vector<pcl::PointCloud<pcl::PointXYZI>> cone_clusters;
        performDBSCANClustering(ground_removed_cloud, cone_clusters, MIN_POINTS, MAX_POINTS);

        // Classify clusters and store classified cones
        std::vector<pcl::PointCloud<pcl::PointXYZI>> classified_cones;
        std::vector<eufs_msgs::msg::ConeWithCovariance> blue_cones;
        std::vector<eufs_msgs::msg::ConeWithCovariance> yellow_cones;
        std::vector<eufs_msgs::msg::ConeWithCovariance> orange_cones;
        std::vector<eufs_msgs::msg::ConeWithCovariance> big_orange_cones;
        std::vector<eufs_msgs::msg::ConeWithCovariance> unknown_color_cones;

        for (auto& cluster : cone_clusters) {
            if (cluster.points.size() >= MIN_POINTS && cluster.points.size() <= MAX_POINTS) {
                // Only keep clusters within point count range
                cone_detection::ConeType cone_type = classifyCone(cluster);

                // Calculate centroid for cone position
                Eigen::Vector4f centroid(0, 0, 0, 0);
                for (const auto& point : cluster.points) {
                    centroid[0] += point.x;
                    centroid[1] += point.y;
                    centroid[2] += point.z;
                }
                centroid /= cluster.points.size();

                // Create a ConeWithCovariance message
                eufs_msgs::msg::ConeWithCovariance cone_msg;
                cone_msg.point.x = centroid[0];
                cone_msg.point.y = centroid[1];
                cone_msg.point.z = centroid[2];

                // Set the covariance matrix (identity matrix with small noise on the last element)
                cone_msg.covariance = {{
                    static_cast<double>(0.0f), static_cast<double>(0.0f), static_cast<double>(0.0f), static_cast<double>(0.000001f) // Identity covariance with small noise
                }};

                // Classify cones and store them in appropriate lists
                if (cone_type == cone_detection::ConeType::BIG_ORANGE) {
                    big_orange_cones.push_back(cone_msg);
                } else if (cone_type == cone_detection::ConeType::BLUE) {
                    blue_cones.push_back(cone_msg);
                } else if (cone_type == cone_detection::ConeType::YELLOW) {
                    yellow_cones.push_back(cone_msg);
                } else if (cone_type == cone_detection::ConeType::ORANGE) {
                    orange_cones.push_back(cone_msg);
                } else {
                    unknown_color_cones.push_back(cone_msg);
                }

                classified_cones.push_back(cluster);
            }
        }

        // Prepare the ConeArrayWithCovariance message
        eufs_msgs::msg::ConeArrayWithCovariance cone_array_msg;
        cone_array_msg.header = msg->header;  // Use the same header as the PointCloud2 message
        cone_array_msg.blue_cones = blue_cones;
        cone_array_msg.yellow_cones = yellow_cones;
        cone_array_msg.orange_cones = orange_cones;
        cone_array_msg.big_orange_cones = big_orange_cones;
        cone_array_msg.unknown_color_cones = unknown_color_cones;

        /* size_t blue_cones_count = blue_cones.size();
        size_t yellow_cones_count = yellow_cones.size();
        size_t orange_cones_count = orange_cones.size();
        size_t big_orange_cones_count = big_orange_cones.size();
        size_t unknown_cones_count = unknown_color_cones.size();

        RCLCPP_INFO(this->get_logger(), "Blue cones count: %lu", blue_cones_count);
        RCLCPP_INFO(this->get_logger(), "Yellow cones count: %lu", yellow_cones_count);
        RCLCPP_INFO(this->get_logger(), "Orange cones count: %lu", orange_cones_count);
        RCLCPP_INFO(this->get_logger(), "Big orange cones count: %lu", big_orange_cones_count);
        RCLCPP_INFO(this->get_logger(), "Unknown color cones count: %lu", unknown_cones_count); */

        // Publish the cone array with covariance
        cone_pub->publish(cone_array_msg);

        // Publish detected cone clusters
        //TODO: Comment this if it is not useful
        publishConePointCloud(classified_cones, msg->header);

        // Publish markers for each classified cone
        //TODO: Comment this if it is not useful
        publishConeMarkers(classified_cones, msg->header);

        // Calculate the callback duration
        rclcpp::Time last_time = rclcpp::Clock(RCL_STEADY_TIME).now();
        //calculateTime(first_time, last_time);
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

            cone_detection::ConeType cone_type = classifyCone(cluster);
                
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
            if (cone_type == cone_detection::ConeType::BIG_ORANGE) {
                marker.scale.x = MARKER_BIG_CONE_RADIUS * 2;
                marker.scale.y = MARKER_BIG_CONE_RADIUS * 2;
                marker.scale.z = MARKER_BIG_CONE_HEIGHT;
                marker.color.r = 1.0f;  // Orange
                marker.color.g = 0.55f;
                marker.color.b = 0.0f;
            } else if (cone_type == cone_detection::ConeType::BLUE) {
                marker.scale.x = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.y = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.z = MARKER_SMALL_CONE_HEIGHT;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;  // Blue
            } else if (cone_type == cone_detection::ConeType::YELLOW) {
                marker.scale.x = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.y = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.z = MARKER_SMALL_CONE_HEIGHT;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;  // Yellow
            } else if (cone_type == cone_detection::ConeType::ORANGE) {
                marker.scale.x = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.y = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.z = MARKER_SMALL_CONE_HEIGHT;
                marker.color.r = 1.0f;  // Orange
                marker.color.g = 0.55f;
                marker.color.b = 0.0f;
            } else {
                marker.scale.x = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.y = MARKER_SMALL_CONE_RADIUS * 2;
                marker.scale.z = MARKER_SMALL_CONE_HEIGHT;
                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 0.5f;  // Unknown color (grey)
            }
            marker.color.a = 0.4; // Opacity

            marker_array.markers.push_back(marker);
        }

        // Clear previous markers (important to remove static-objects noise)
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.insert(marker_array.markers.begin(), clear_marker);

        marker_publisher_->publish(marker_array);


        //TODO: Comment if not useful
        //rclcpp::Time timestamp_published = rclcpp::Clock(RCL_STEADY_TIME).now();
        // Write timestamps to file using the message's timestamp as received time
        //writeTimestampsToFile(rclcpp::Time(header.stamp), timestamp_published);
    }

    void writeTimestampsToFile(const rclcpp::Time& timestamp_received, const rclcpp::Time& timestamp_published) {
        std::ofstream file("timestamps.txt", std::ios::app); // Open in append mode
        if (file.is_open()) {
            file << "Received: " << timestamp_received.seconds() << "s\n";
            file << "Published: " << timestamp_published.seconds() << "s\n";
            file << "------------------------------------------\n";
            file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open timestamps.txt for writing.");
        }
    }

    void calculateTime(const rclcpp::Time& start, const rclcpp::Time& end) {
        rclcpp::Duration duration = end - start;
        RCLCPP_INFO(this->get_logger(), "Time difference: %f seconds", duration.seconds());

        std::ofstream file("algo_time.txt", std::ios::app); // Open in append mode
        if (file.is_open()) {
            file << "Time: " << duration.seconds() << "s\n";
            file << "------------------------------------------\n";
            file.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open .txt for writing.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr restricted_fov_publisher_;
    rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr cone_pub;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    
    rclcpp::spin(std::make_shared<ConeDetectionNode>());
    rclcpp::shutdown();

    
    return 0;
}