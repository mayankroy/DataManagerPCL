// This is Cloud veiwing headerfile Customized from Point Cloud tutorial on PCL viewer.
// Mayank Roy - mr2234@cornell.edu

#include "stdafx.h"

//Visualization related libraries
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

class PCLViewer{	

public:
	bool simple, rgb, custom_c, normals, normalsIntegrated, PCI,shapes, viewports, interaction_customization,  viewerOn;
	unsigned int text_id;
	bool startViewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	PCLViewer()
	{                                     //Initializing Parameters

		viewerOn = false;
		simple=false;
		rgb= false;
		custom_c=false;
		normals = false;
		normalsIntegrated = false;
		PCI = false;
	    shapes = false; 
		viewports = false;
		interaction_customization = false;
		startViewer = false;
		text_id = 0;
	}

int parse_cloud(pcl::PointCloud<pcl::PointXYZRGBNormal> scene, pcl::PointCloud<pcl::PrincipalCurvature> curvatureVec)

int start_viewer(int argc, char** argv, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  else if (pcl::console::find_argument (argc, argv, "-s") >= 0)
  {
    simple = true;
    std::cout << "Simple visualisation example\n";
    viewer = simpleVis(cloud);
  }
  else if (pcl::console::find_argument (argc, argv, "-r") >= 0)
  {
    rgb = true;
    viewer = rgbVis(colored_cloud); //With Surface Patching
    std::cout << "RGB colour visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-n") >= 0)
  {
    normals = true;
    viewer = normalsVis(voxel, cloud_normals); //With points green colour on basic pointcloud and normals
    std::cout << "Normals visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-pci") >= 0)
  {
    PCI = true;
    viewer = normalsVis(voxel, cloud_normals); //With points green colour on basic pointcloud and normals
    std::cout << "PCI patch visualisation example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-v") >= 0)
  {
    viewports = true;
    std::cout << "Viewports example\n";
  }
  else if (pcl::console::find_argument (argc, argv, "-i") >= 0)
  {
    interaction_customization = true;
    std::cout << "Interaction Customization example\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  while (!viewer->wasStopped () && startViewer)
  { 
	viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  viewer->close();
  startViewer = false;
  viewerOn = false;  
  cout<<"Viewer Closed"<<endl;
}


void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-pci         PCI (patch)visualisation example\n"
            << "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simple (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pciVis (
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 1, "normals");
  viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal> (cloud, normals, principalCurvatures, 1, 1, "pc");
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals1, pcl::PointCloud<pcl::Normal>::Ptr normals2)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();
  viewer->setCameraPosition (1, 1,1, 0,0,0, 0,0,1,0);

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}



};

