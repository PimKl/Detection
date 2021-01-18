#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

// Point Type
// pcl::PointXYZ, pcl::PointXYZI, pcl::PointXYZRGBA
typedef pcl::PointXYZI PointType;




//----------------------------------------------------------------------------------------
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  Car egoCar(Vect3(0,  0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(  Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(  Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(  Vect3(-12,4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}


/// @brief Open 3D viewer and display City Block
void cityBlock(
  pcl::visualization::PCLVisualizer::Ptr&   viewer,
  ProcessPointClouds<pcl::PointXYZI>* const pointProcessorI,
  pcl::PointCloud<pcl::PointXYZI>::Ptr&     inputCloud)
{
  // Step 0: Load PCD-file
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud =
  //  pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  // Step 1: Apply voxel grid filter
  inputCloud = pointProcessorI->FilterCloud(
    inputCloud,
    0.3 ,
    Eigen::Vector4f (-20, -20, -5, 1),
    Eigen::Vector4f ( 20, 20, 5, 1));
  //renderPointCloud(viewer, inputCloud, "inputCloud");

/*
  // Step 2: Segment the filtered cloud into two parts, road and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud =
    pointProcessorI->RansacPlane(inputCloud, 100, 0.1);

  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
  renderPointCloud(viewer, segmentCloud.first,  "obstCloud", Color(1, 0, 0));
*/

  // Step 3: Cluster the obstacle cloud
  KdTree* tree = new KdTree;
  for (int i = 0; i < /*segmentCloud.first*/inputCloud->points.size(); ++i) {
    tree->insert(/*segmentCloud.first*/inputCloud->points[i], i);
  }
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->euclideanCluster(
    /*segmentCloud.first*/inputCloud, tree, 0.35, 15, 500);

  int clusterId = 0;

  std::vector<Color> colors = {
    Color(1, 0, 0),
    Color(1, 1, 0),
    Color(0, 0, 1)
  };

  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    // Render points of the cluster
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // Step 4: Render bounding box around cluster
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId;
  }

  delete tree;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(
  CameraAngle                             setAngle,
  pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  viewer->setBackgroundColor (0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch(setAngle)
  {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle!=FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}


/*

int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped ()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load PCD and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    ++streamIterator;
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }

    viewer->spinOnce();
  }

  delete pointProcessorI;

  return 0;
}


*/


//------------------------------------------------------------------------------------------------



int main( int argc, char *argv[] )
{
    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) ){
        std::cout << "usage: " << argv[0]
            << " [-ipaddress <192.168.1.70>]"
            << " [-port <2368>]"
            << " [-pcap <*.pcap>]"
            << " [-help]"
            << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Point Cloud Color Hndler
    pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
    const std::type_info& type = typeid( PointType );
    if( type == typeid( pcl::PointXYZ ) ){
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<PointType>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<PointType>() );
        handler = color_handler;
    }
    else{
        throw std::runtime_error( "This PointType is unsupported." );
    }

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );

            // Point Cloud Processing 

            cloud = ptr;
        };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    if( !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ){
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );

    }

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud

            handler->setInputCloud( cloud );
	    
	    pcl::io::savePCDFile ("test_pcd.pcd", *cloud, true);

	    viewer->removeAllPointClouds();
	    viewer->removeAllShapes();

	    inputCloudI = pointProcessorI->loadPcd( "test_pcd.pcd" );
    	    cityBlock(viewer, pointProcessorI, inputCloudI);
 		
	    
            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
		
				

                viewer->addPointCloud( cloud, *handler, "cloud" );

		std::cout << "test3 " << std::endl;
            }
	    
	   
        }

	

	//viewer->spinOnce();
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}


