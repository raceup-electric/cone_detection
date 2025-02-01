#include "cone_detection/cone_clustering.hpp"
#include "cone_detection/cone_type.hpp"

// DBSCAN parameters
const float EPS_subCluster = 0.15;      // Cluster tolerance (distance)
const float tolerance = 0.1; //tolerance on height of clusters that are too big


void performDBSCANClustering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters, int min_points, int max_points, float eps) {
        // Parameters for the number of points in a cluster

        // Build a KD-tree for clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        tree->setInputCloud(input_cloud.makeShared());

        // Perform DBSCAN-based clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(eps);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input_cloud.makeShared());
        ec.extract(cluster_indices);

        // Store each cluster in a separate PointCloud
        for (const auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZI> cluster;

            // Max and min coordinates of points
            float min_x = input_cloud.points[indices.indices[0]].x;
            float max_x = input_cloud.points[indices.indices[0]].x;
            float min_y = input_cloud.points[indices.indices[0]].y;
            float max_y = input_cloud.points[indices.indices[0]].y;
            float min_z = input_cloud.points[indices.indices[0]].z;
            float max_z = input_cloud.points[indices.indices[0]].z;
            
            // Add points to the cluster and update max/min values
            for (const auto& index : indices.indices) {
                cluster.points.push_back(input_cloud.points[index]);

                if (input_cloud.points[index].x < min_x) min_x = input_cloud.points[index].x;
                if (input_cloud.points[index].x > max_x) max_x = input_cloud.points[index].x;
                if (input_cloud.points[index].y < min_y) min_y = input_cloud.points[index].y;
                if (input_cloud.points[index].y > max_y) max_y = input_cloud.points[index].y;
                if (input_cloud.points[index].z < min_z) min_z = input_cloud.points[index].z;
                if (input_cloud.points[index].z > max_z) max_z = input_cloud.points[index].z;
            }

            // Check horizontal and vertical extents:
            float extent_x = max_x - min_x;
            float extent_y = max_y - min_y;
            float extent_z = max_z - min_z;

            if ((extent_x < 2*cone_detection::BIG_CONE_BASE_RADIUS) && (extent_y < 2*cone_detection::BIG_CONE_BASE_RADIUS)) {
                cone_clusters.push_back(cluster);
            }
            else if ((extent_z <= cone_detection::BIG_CONE_MAX_HEIGHT + tolerance) && eps!=EPS_subCluster)
            {
                performDBSCANClustering(cluster, cone_clusters, min_points, max_points, EPS_subCluster);
            }
        }
    }
