/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

using pclVisualizerPtr = pcl::visualization::PCLVisualizer::Ptr;

std::vector<Car> initHighway(bool renderScene, pclVisualizerPtr &viewer)
{
    std::vector<Car> cars
    {
        Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar"),
        Car(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1"),
        Car(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2"),
        Car(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3")
    };

    if (renderScene)
    {
        renderHighway(viewer);
        for (auto &car : cars)
        {
            car.render(viewer);
        }
    }

    return cars;
}

void cityBlock(pclVisualizerPtr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // filtering and cropping
    inputCloud = pointProcessor.FilterCloud(inputCloud, 1, Eigen::Vector4f(-10,-10,-2,1), Eigen::Vector4f(30,8,2,1));
    // excluding the carroof
    Box carRoof {-2,-1.5,-1,2.75,1.5,0.25};
    renderBox(viewer, carRoof, 0, Color(0,0,1), 0.5);
    inputCloud = pointProcessor.ExcludeCloud(
        inputCloud, 
        Eigen::Vector4f(carRoof.x_min,carRoof.y_min,carRoof.z_min,1), 
        Eigen::Vector4f(carRoof.x_max,carRoof.y_max,carRoof.z_max,1)
        );

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> 
        segmentCloud = pointProcessor.SegmentPlane(inputCloud, 500, 0.3);

    auto cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.5, 3, 30);

    int clusterId = 1; // 0 is for the carRoof
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (auto cluster : cloudClusters)
    {
        auto c = colors[clusterId%colors.size()];
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), c);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, c, 0.5);
        ++clusterId;
    }
    
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
}

void simpleHighway(pclVisualizerPtr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    auto lidar = std::make_unique<Lidar>(cars, 0);
    auto inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");
    // TODO:: Create point processor
    auto pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();
    auto segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    auto cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for (auto cluster : cloudClusters)
    {
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pclVisualizerPtr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : 
        default: viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // viewer is a boost sharedptr
    auto viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}