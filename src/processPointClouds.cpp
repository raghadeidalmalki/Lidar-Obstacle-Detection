// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include <vector>

// Constructor
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// Destructor
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

// Print number of points in the cloud
template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// Filter cloud using voxel grid and crop box
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloudVGFiltered{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudVGFiltered);

    // Region of Interest (ROI) based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudBoxFiltered{new pcl::PointCloud<PointT>};
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudVGFiltered);
    cropBoxFilter.filter(*cloudBoxFiltered);

    // Get the indices of rooftop points
    std::vector<int> indices;
    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(cloudBoxFiltered);
    roofFilter.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    // Remove the rooftop indices
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(cloudBoxFiltered);
    extract.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;
}

// Separate point cloud into obstacles and planes
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

// Segment the largest planar component from the input cloud
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
  
    // TODO:: Fill in this function to find inliers for the cloud.
 pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
    pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    pcl::SACSegmentation<PointT> seg;
  


	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	// Segment the largest planar component from the input cloud
	seg.setInputCloud(cloud);
	seg.segment (*inliers, *coefficients);
  
  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
     pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices());
    srand(static_cast<unsigned int>(time(nullptr)));

    // Segment the planar component from the cloud, represented by indices of inliers from fitted plane with most inliers
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit a plane
        std::unordered_set<int> sampleIndices;
        while (sampleIndices.size() < 3)
        {
            sampleIndices.insert(rand() % cloud->points.size());
        }

        auto itr = sampleIndices.begin();
        PointT point1 = cloud->points[*itr];
        itr++;
        PointT point2 = cloud->points[*itr];
        itr++;
        PointT point3 = cloud->points[*itr];

        // Fit a plane, Ax+By+Cz+D=0
        float A = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        float B = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        float C = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        float D = -1 * (A * point1.x + B * point1.y + C * point1.z);

        // Measure distance between every point and fitted plane
        pcl::PointIndices::Ptr inliersTemp(new pcl::PointIndices());
        for (size_t idx = 0; idx < cloud->points.size(); ++idx)
        {
            PointT point = cloud->points[idx];
            float distance = fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceThreshold)
            {
                inliersTemp->indices.push_back(idx);
            }
        }

        if (inliersTemp->indices.size() > inliersResult->indices.size())
        {
            inliersResult = inliersTemp;
        }
    }

    if (inliersResult->indices.empty())
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
    return segResult;
}



// Custom clustering algorithm based on KdTree and distance-based approach
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    // Create a KdTree object for the search method
    KdTree* tree = new KdTree;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        PointT point = cloud->points[i];
        tree->insert({point.x, point.y, point.z}, i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processedPoints(cloud->points.size(), false);

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        if (processedPoints[i])
            continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        clusterHelper(cloud, processedPoints, i, cluster, tree, clusterTolerance);

        // Only keep the cluster with a size between the allowance
        if ((cluster->size() >= minSize) && (cluster->size() <= maxSize))
        {
            cluster->width = cluster->size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }

    delete tree;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// Helper function for clustering
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& processedPoints, int index, typename pcl::PointCloud<PointT>::Ptr cluster, KdTree* tree, float clusterTolerance)
{
    processedPoints[index] = true;
    cluster->push_back(cloud->points[index]);

    PointT point = cloud->points[index];
    std::vector<int> proximity = tree->search({point.x, point.y, point.z}, clusterTolerance);
    for (int id : proximity)
    {
        if (!processedPoints[id])
        {
            clusterHelper(cloud, processedPoints, id, cluster, tree, clusterTolerance);
        }
    }
}

// Bounding box for clusters
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
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

// Save point cloud to file
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

// Load point cloud from file
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

// Stream PCD files
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    sort(paths.begin(), paths.end());

    return paths;
}
