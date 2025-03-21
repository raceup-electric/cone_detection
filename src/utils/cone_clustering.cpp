#include "cone_detection/cone_clustering.hpp"
#include "cone_detection/cone_type.hpp"
#include "dbscan/algo.h"

// DBSCAN parameters
const float EPS_subCluster = 0.1;      // Cluster tolerance (distance)
const float tolerance = 0.1; //tolerance on height of clusters that are too big
const float min_subCluster_points_ratio = 0.1; // Minimum number of points in a subcluster, expressed as a fraction of the main cluster's points


void performDBSCANClustering(const pcl::PointCloud<pcl::PointXYZI>& input_cloud,
                             std::vector<pcl::PointCloud<pcl::PointXYZI>>& cone_clusters, int min_points, int max_points, float eps, float EPS_subClustering) {

        int n = input_cloud.points.size(); // number of data points
        std::vector<double> data(n*3); // data points
        std::vector<int> labels(n); // label ids get saved here
        std::vector<char> core_samples(n); // a flag determining whether or not the sample is a core sample is saved here
        std::vector<int> core_point(n);

        // Populate the 'data' vector with point cloud coordinates
        for (int i = 0; i < n; i++) {
            data[i*3 + 0] = input_cloud.points[i].x;
            data[i*3 + 1] = input_cloud.points[i].y;
            data[i*3 + 2] = input_cloud.points[i].z;
        }

        // Perform DBSCAN clustering
        DBSCAN<3>(n, data.data(), eps, min_points, (bool*)core_samples.data(), core_point.data(), labels.data());

        // Group points by their assigned cluster labels
        std::unordered_map<int, pcl::PointIndices> cluster_indices;
        for (int i = 0; i < n; i++) {
            if(labels[i] == -1) continue; //invalid point
            cluster_indices[labels[i]].indices.push_back(i);
        }

        // Store each cluster in a separate PointCloud
        for (const auto& [key, indices] : cluster_indices) {
            if(indices.indices.size() > max_points) continue;
            
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

            if ((extent_z <= cone_detection::BIG_CONE_MAX_HEIGHT + tolerance) && (extent_z >= tolerance) ) {
                if ((extent_x < 2*cone_detection::BIG_CONE_BASE_RADIUS + tolerance) && (extent_y < 2*cone_detection::BIG_CONE_BASE_RADIUS + tolerance)) {
                    cone_clusters.push_back(cluster);
                }
                else if (eps != EPS_subCluster)
                {
                    performDBSCANClustering(cluster, cone_clusters, indices.indices.size()*min_subCluster_points_ratio, max_points, EPS_subCluster, EPS_subClustering);
                }
            }
        }
    }
