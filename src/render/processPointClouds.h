// PCL lib Functions for processing point clouds

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include "box.h"
#include <ctime>
#include <filesystem>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <type_traits>
#include <vector>

template <typename PointT> class ProcessPointClouds
{
  public:
    // constructor
    ProcessPointClouds()
    {
    }
    // deconstructor
    ~ProcessPointClouds()
    {
    }

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        std::cout << cloud->points.size() << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
                                                      Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
    {

        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();

        // TODO:: Fill in the function to do voxel grid point reduction and region
        // based filtering

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

        return cloud;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)

    {
        // TODO: Create two new point clouds, one cloud with obstacles and other with
        // segmented plane

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
        return segResult;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
    {
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        pcl::PointIndices::Ptr inliers;
        // TODO:: Fill in this function to find inliers for the cloud.

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult =
            SeparateClouds(inliers, cloud);
        return segResult;
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                  float clusterTolerance, int minSize, int maxSize)
    {

        // Time clustering process
        auto startTime = std::chrono::steady_clock::now();

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        // TODO:: Fill in the function to perform euclidean clustering to group
        // detected obstacles

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
                  << " clusters" << std::endl;

        return clusters;
    }

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
    {

        // Find bounding box for one of the clusters
        PointT minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);

        Box box;
        box.x_min = minPoint.x;
        box.y_min = minPoint.y;
        box.z_min = minPoint.z;
        box.x_max = maxPoint.x;
        box.y_max = maxPoint.y;
        box.z_max = maxPoint.z;

        return box;
    }

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
    {
        pcl::io::savePCDFileASCII(file, *cloud);
        std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file)
    {

        typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

        if(pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file \n");
        }
        std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

        return cloud;
    }

    std::vector<std::filesystem::path> streamPcd(std::string dataPath)
    {

        std::vector<std::filesystem::path> paths(std::filesystem::directory_iterator{std::filesystem::path{dataPath}},
                                                 std::filesystem::directory_iterator{});

        // sort files in accending order so playback is chronological
        sort(paths.begin(), paths.end());

        return paths;
    }
};
#endif /* PROCESSPOINTCLOUDS_H_ */
