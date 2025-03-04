// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <thread>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
//Generating cars
 Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
  
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
  // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
  

    // Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
    // renderRays(viewer, lidar->position, pointCloud);
    // renderPointCloud(viewer, pointCloud, "pointCloud"); 

  	//renderPointCloud(viewer, inputCloud, "inputCloud" ); // to view the lidar's point cloud by itself, without the rays, we call renderPointCloud instead of renderRays
  
  
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // Segment the point cloud
   
  
  
   // Segment the point cloud
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pointCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

   
  
  

    // Euclidean Clustering 
  //Cluster the obstacles
  //start of my code
  // Cluster the obstacles
   
  
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};

  
  
  
  
  
  
   // Euclidean Clustering 
//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
//int clusterId = 0;
  //  Color boxColor(0, 1, 0);  // Green color for the bounding boxes
  
  
  
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstcleCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
   }

  
    
}

//end of my code





/* For single PCD */
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
/* For streaming PCD */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)

{
  
  
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // Create point processor with intensity
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");
  
  
      // TODO: use leaf size of 0.2m, experiment with the other hyperparameters
    // Set the min/max Eigen::Vector4f(x, y, z, 1.0) values
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.18, Eigen::Vector4f(-10, -5.0, -2, 1), Eigen::Vector4f(30, 7.0, 4, 1));
     //renderPointCloud(viewer, filteredCloud, "filteredCloud");
  
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.2);
  
 
  
  
  
  
    //  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
  
  
 
  
   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneRansac(filteredCloud, 100, 0.2);

  renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
  
  
  
  // Cluster the obstacle cloud (initialize cloudClusters here)
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 600);  
 
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->ClusteringEuclidean(segmentCloud.first, 0.5, 10, 600);

  
 
 
   int clusterId = 0;
   std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(1,1,0)};

// Loop over each cluster and render them with bounding boxes
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster " << std::to_string(clusterId) << " has a size of ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Compute bounding box and render it
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }

 
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)

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
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

  
int main (int argc, char** argv)
{
 

    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    //simpleHighway(viewer);
    //cityBlock(viewer);
  
   



    while (!viewer->wasStopped ())
    {
      
       // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        viewer->spinOnce ();
    } 
}