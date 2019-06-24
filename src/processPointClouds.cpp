// PCL lib Functions for processing point clouds 

#include <random>
#include <Eigen/Core>
#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    auto obstCloud = boost::make_shared<pcl::PointCloud<PointT>>();
    auto planeCloud = boost::make_shared<pcl::PointCloud<PointT>>();

    for (auto index: inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceToPlane, float& a, float& b, float& c, float& d)
{
	std::unordered_set<int> inlier;

	while(--maxIterations)
	{
		// random choose samples to construct model
		std::unordered_set<int> sampleSet;

		std::random_device rd{};
    	std::mt19937 rg{rd()};
 
    	// values near the mean are the most likely
    	// standard deviation affects the dispersion of generated values from the mean
   	 	std::uniform_int_distribution<> uDistribution(0,cloud->size()-1); // template< class RealType = double >

		while (sampleSet.size() < 3)
			sampleSet.insert(uDistribution(rg));

		float x[3], y[3], z[3];
		auto itr = std::begin(sampleSet);
		for (size_t i = 0; i < 3; ++i)
		{
			x[i] = cloud->points[*itr].x;
			y[i] = cloud->points[*itr].y;
			z[i] = cloud->points[*itr].z;
			++itr;
 		}

		Eigen::Vector3f v1 = {x[2] - x[1], y[2] - y[1], z[2] - z[1]};
		Eigen::Vector3f v2 = {x[2] - x[0], y[2] - y[0], z[2] - z[0]};

		Eigen::Vector3f normal = v1.cross(v2);
		normal = normal.normalized();

		float sa = normal[0];
		float sb = normal[1];
		float sc = normal[2];

		float sd = -(sa*x[0]+sb*y[0]+sc*z[0]);

		// test the rest of samples
		for(int i = 0; i < cloud->points.size(); ++i)
		{
			if (sampleSet.count(i) > 0)
			{
				continue;
			}

			float x = cloud->points[i].x;
			float y = cloud->points[i].y;
			float z = cloud->points[i].z;

			float d = fabs(sa*x + sb*y + sc*z + sd)/sqrt(sa*sa + sb*sb + sc*sc);

			if (d < distanceToPlane)
			{
				sampleSet.insert(i);
			}
		}

		if (sampleSet.size() > inlier.size())
		{
			inlier = sampleSet;
			a = sa;
			b = sb;
			c = sc;
			d = sd;
		}
	}
	
	return inlier;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::SACSegmentation<PointT> seg;
	auto inliers = boost::make_shared<pcl::PointIndices>();
    auto coefficients = boost::make_shared<pcl::ModelCoefficients>();

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cout << "could not estimate a plannar model for the given dataset. " << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    auto tree = boost::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> extractor;
    extractor.setClusterTolerance(clusterTolerance);
    extractor.setMinClusterSize(minSize);
    extractor.setMaxClusterSize(maxSize);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(clusterIndices);

    for(const auto& getIndices : clusterIndices)
    {
        auto cloudCluster = boost::make_shared<pcl::PointCloud<PointT>>();

        for (auto idx : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[idx]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
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


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}