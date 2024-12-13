#include "cone_detection/cone_clustering.hpp"

// DBSCAN parameters
const float EPS = 0.5;                       // Cluster tolerance (distance)


void performDBSCANClustering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters, int min_points, int max_points) {
        // Parameters for the number of points in a cluster

        // Build a KD-tree for clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(input_cloud.makeShared());

        // Perform DBSCAN-based clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(EPS);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud.makeShared());
        ec.extract(cluster_indices);

        // Store each cluster in a separate PointCloud
        for (const auto& indices : cluster_indices) {
            // Check if the cluster has the desired number of points
            if (indices.indices.size() >= min_points && indices.indices.size() <= max_points) {
                pcl::PointCloud<pcl::PointXYZI> cluster;
                for (const auto& index : indices.indices) {
                    cluster.points.push_back(input_cloud.points[index]);
                }
                cone_clusters.push_back(cluster);
            }
        }
    }

