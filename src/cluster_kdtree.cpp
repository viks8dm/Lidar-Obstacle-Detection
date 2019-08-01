#include <chrono>
#include <string>
#include "kdtree_pcl.h"

template<typename PointT>
class ClusterPts {
private:
  int num_points;
  float clusterTol;
  int minClusterSize;
  int maxClusterSize;
  std::vector<bool> processed;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // std::vector<std::vector<int>> clusters;

public:
  ClusterPts(int nPts, float clustTol, int minSize, int maxSize) :
      num_points(nPts), clusterTol(clustTol), minClusterSize(minSize), maxClusterSize(maxSize) {
        processed.assign(num_points, false);
      }
  ~ClusterPts();

  void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int idx);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclidCluster(typename pcl::PointCloud<PointT>::Ptr cloud);
};


/**********************************************************************/
template<typename PointT>
ClusterPts<PointT>::~ClusterPts() {}

/**********************************************************************/
template<typename PointT>
void ClusterPts<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, std::vector<int>& cluster, int idx) {
  processed[idx] = true;
  cluster.push_back(idx);

  std::vector<int> neighbor_point = tree->search(cloud->points[idx], clusterTol);
  for(int neighbor_id: neighbor_point) {
    if (!processed[neighbor_id]) {
      proximity(cloud, tree, cluster, neighbor_id);
    }
  }
}

/**********************************************************************/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusterPts<PointT>::EuclidCluster(typename pcl::PointCloud<PointT>::Ptr cloud) {

  // std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  KdTree* tree = new KdTree;

  for (int i=0; i<num_points; i++)
    tree->insert(cloud->points[i],i);

  for (int i=0; i<num_points; i++) {
    if (processed[i]) {
      i++;
      continue;
    }

    std::vector<int> cluster_idx;
    typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
    proximity(cloud, tree, cluster_idx, i);

    int cluster_size = cluster_idx.size();

    if (cluster_size>=minClusterSize && cluster_size<=maxClusterSize) {
      for (int j=0; j<cluster_size; j++) {
        cloudCluster->points.push_back(cloud->points[cluster_idx[j]]);
      }

      cloudCluster->width = cloudCluster->points.size();
			cloudCluster->height = 1;

      clusters.push_back(cloudCluster);
    }

  }

  return clusters;
}
