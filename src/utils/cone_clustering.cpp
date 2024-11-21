#include "cone_detection/cone_clustering.hpp"

// DBSCAN parameters
const float EPS = 0.5;                       // Cluster tolerance (distance)

const int MIN_CLUSTER_SIZE = 20;  // Minimum number of points for a cluster to be considered valid
const int MAX_CLUSTER_SIZE = 200; // Maximum number of points for a cluster to be considered valid

void performDBSCANClustering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters, int min_points) {
        // Parameters for the number of points in a cluster

        // Build a KD-tree for clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(input_cloud.makeShared());

        // Perform DBSCAN-based clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(EPS);
        ec.setMinClusterSize(min_points);
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