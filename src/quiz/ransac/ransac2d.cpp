/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include <random>
#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateLineData()
{
	auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; ++i)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	auto viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("2D Viewer");
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol, float& a, float& b, float& c)
{
	srand(time(NULL));
	std::unordered_set<int> inlier;

	while(--maxIterations)
	{
		// random choose samples to construct model
		std::unordered_set<int> sampleSet;
		while (sampleSet.size() < 2)
		{
			sampleSet.insert(rand()%(cloud->points.size()));
		}

		float x1, y1, x2, y2;

		auto itr = sampleSet.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		++itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float sa = (y2 - y1);
		float sb = (x1 - x2);
		float sc = y1 * (x2-x1) - x1 * (y2-y1);

		// test other samples
		for(int index = 0; index < cloud->points.size(); ++index)
		{
			if (sampleSet.count(index) > 0)
			{
				continue;
			}

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(sa*x3 + sb*y3 + sc)/sqrt(sa*sa + sb*sb);

			if (d <= distanceTol)
			{
				sampleSet.insert(index);
			}
		}

		if (sampleSet.size() > inlier.size())
		{
			inlier = sampleSet;
			a = sa;
			b = sb;
			c = sc;
		}
	}
	
	return inlier;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol, float& a, float& b, float& c, float& d)
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

			if (d <= distanceTol)
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	float a, b, c, d;
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.5, a, b, c, d);

	auto cloudInliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	auto cloudOutliers = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

	for(int i = 0; i < cloud->points.size(); ++i)
	{
		pcl::PointXYZ point = cloud->points[i];
		if(inliers.count(i))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
