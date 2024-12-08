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


const float LIDAR_HEIGHT = 0.6;
const float HEIGHT_FILTER = 0.8;
const float MAX_HEIGHT_THRESHOLD = HEIGHT_FILTER - LIDAR_HEIGHT;  // Maximum allowed height for all points (non-ground points)
const float DISTANCE_RADIUS_THRESHOLD = 10.0;       // Distance threshold for filtering

const int MIN_POINTS = 10;                   // Minimum points per cluster

// Cone size constraints for RVIZ2 Markers
const float MARKER_SMALL_CONE_HEIGHT = 0.3;
const float MARKER_SMALL_CONE_RADIUS = 0.2;
const float MARKER_BIG_CONE_HEIGHT = 0.6;
const float MARKER_BIG_CONE_RADIUS = 0.3;

class ConeDetectionNode : public rclcpp::Node {
public:
    ConeDetectionNode() : Node("cone_detection_node") {
        pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/ouster/points", 10,
                std::bind(&ConeDetectionNode::pointCloudCallback, this, std::placeholders::_1));

        cone_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_cones", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_markers", 10);

        restricted_fov_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/restricted_fov", 10);

    }

private:

    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*                                   ROS2 CALLBACK FOR LIDAR DATA                                        */
    /*********************************************************************************************************/
    /*********************************************************************************************************/

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 to PCL PointCloud<PointXYZI> for processing
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        //Filter out points based on their coordinates
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
        restrictedFOVFiltering(pcl_cloud, filtered_cloud, MAX_HEIGHT_THRESHOLD);

        // PUBLISH THE RESTRICTED FOV
        sensor_msgs::msg::PointCloud2 restricted_fov_msg;
        pcl::toROSMsg(filtered_cloud, restricted_fov_msg);
        restricted_fov_msg.header = msg->header;
        restricted_fov_publisher_->publish(restricted_fov_msg);


        // Separate ground and non-ground points using RANSAC
        pcl::PointCloud<pcl::PointXYZI> ground_removed_cloud;
        removeGroundRANSAC(filtered_cloud, ground_removed_cloud);


        // Cluster the remaining points
        std::vector<pcl::PointCloud<pcl::PointXYZI>> cone_clusters;
        performDBSCANClustering(ground_removed_cloud, cone_clusters, MIN_POINTS);

        // Classify clusters and store classified cones
        std::vector<pcl::PointCloud<pcl::PointXYZI>> classified_cones;
        for (auto& cluster : cone_clusters) {
            if (cluster.points.size() >= MIN_POINTS && cluster.points.size() <= 25000) {
                // Only keep clusters within point count range
                cone_detection::ConeType cone_type = classifyCone(cluster);
                if (cone_type != cone_detection::ConeType::UNKNOWN) {
                    classified_cones.push_back(cluster);  // Only store recognized cones
                }
            }
        }

        // Publish detected cone clusters
        publishConePointCloud(classified_cones, msg->header);

        // Publish markers for each classified cone
        publishConeMarkers(classified_cones, msg->header);
    }

    /*********************************************************************************************************/
    /*********************************************************************************************************/
    /*                                        PUBLISHING METHODS                                             */
    /*********************************************************************************************************/
    /*********************************************************************************************************/


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
            marker.color.a = 1.0; // Full opacity

            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr restricted_fov_publisher_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
