#define PI 3.14159265

#include <iostream>
#include <math.h> 
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>

// downSampling
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// filtering
#include <pcl/filters/statistical_outlier_removal.h>

//Voxelize
#include <pcl/filters/passthrough.h>

//using namespace std;
// Base Height = 6mm
// Top Height = 91mm
// Length = 215mm
// Bredth = 134mm

class PCLdata{	

public:
	

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene;
	pcl::PointCloud<pcl::PointXYZ>::Ptr scenePoints;
	pcl::PointCloud<pcl::PointNormal>::Ptr sceneNormals;
	pcl::PointCloud<pcl::PrincipalCurvature>::Ptr curvatureVectors;  
	Eigen::Matrix<float, 3, 4> projectionMat;
	Eigen::Matrix3f CI;
	int backProjection[2056][2454];
	//World Coordinates

	PCLdata()
	{                                
		//Initializing Parameters
		scene = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGNormal>);
	        scenePoints = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		sceneNormals = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>); 
		curvatureVectors = pcl::PointCloud<pcl::PrincipalCurvature>::Ptr (new pcl::PointCloud<pcl::PrincipalCurvature>);
		projectionMat<<  -0.9966,    0.0214,   -0.0797,  0.5487686,
                                  0.0233,    0.9995,   -0.0232,  -0.0858538,
                                  0.0792,   -0.0249,   -0.9965,  0.2873345;
		
		//CI<< 2176.211068, 0.000000, 1244.289249, 0.000000, 2178.633804, 1041.836938, 0.000000, 0.000000, 1.000000;

	}


int readOBJ();
int readPLY();
int readPCD();
//Create Dense Point Cloud from OBJ textured file
void createDenseCloud( int ctr, double (*arr)[1000][1280][8])
{

//%   calc of R|T matrix
//Rx=[1 0 0;0 cosd(c) -sind(c);0 sind(c) cosd(c)];
//Ry=[cosd(b) 0 sind(b); 0 1 0;-sind(b) 0 cosd(b)];
//Rz=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1];
//Q=Rz*Ry*Rx;
//T=[tx;ty;tz];
//B_T_T=[Q T;0 0 0 1];%Base with respt to tool matrix
	
/*pcl::PointXYZ basic_point;
for (int profile_no=0;profile_no<ctr;profile_no++)
{
for(unsigned int i=0; i<1280; i++)
{
basic_point.x = (*arr)[profile_no][i][0];
basic_point.y = (*arr)[profile_no][i][2];
basic_point.z = (*arr)[profile_no][i][1];
cloud->points.push_back(basic_point);
}
}*/
    cout<<"transforming cloud"<<endl;	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ basic_point;

    cout<<"transforming cloud"<<endl;
    cloud->width = (int) cloud->points.size ();
    cloud->height = 1;
    cout<<*cloud<<endl;
    basic_boat = cloud;
    //pcl::io::loadPCDFile ("my_point_cloud.pcd", basic_boat);	
    //cout<<"writing cloud"<<endl;	
    //pcl::io::savePCDFileASCII ("my_point_cloud_trans.pcd", *cloud);
    /*pcl::PCDWriter writer;
    writer.write ("my_point_cloud.pcd", *cloud);*/
    //cout<<"writing cloud"<<endl;	
}


int parseCloud();

void  desample()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr resampled_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = basic_boat;
    //desampling
    pcl::PCLPointCloud2::Ptr cloud_2 (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr resampled_cloud_2 (new pcl::PCLPointCloud2 ());

    pcl::toPCLPointCloud2(*cloud, *cloud_2);
    //pcl::toPCLPointCloud2(*resampled_cloud, *resampled_cloud_2);
   
    //// Fill in the cloud data
    //pcl::PCDReader reader;
    //// Replace the path below with the path where you saved your file
    //reader.read ("test_pcd.pcd", *cloud); // Remember to download the file first!
    std::cout << "PointCloud before Desampling: " << cloud_2->width * cloud_2->height << " data points (" << pcl::getFieldsList (*cloud) << ").";
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud_2);
    sor.setLeafSize (0.001f, 0.001f, 0.001f);
    sor.filter (*resampled_cloud_2);
    //fromPCLPointCloud2( *cloud_2, *cloud);
    fromPCLPointCloud2(*resampled_cloud_2, *resampled_cloud);
    std::cout<< "PointCloud after Desampling: " << resampled_cloud->width * resampled_cloud->height << " data points (" << pcl::getFieldsList (*resampled_cloud) << ").";
    resampled_boat = resampled_cloud;
    /* pcl::PCDWriter writer;
writer.write ("my_point_cloud-ds.pcd", *resampled_cloud);*/
 
}

void  filterOuliers()
{
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
cloud = resampled_boat;

std::cout << "Cloud before filtering: " << std::endl;
std::cout << *cloud << std::endl;
// Create the filtering object
//Filtering based on mean radius
//pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud (cloud);
sor.setMeanK (50);
sor.setStddevMulThresh (0.4);
sor.filter (*filtered_cloud);
std::cout << "Cloud after filtering: " << std::endl;
std::cout << *filtered_cloud << std::endl;
filtered_boat = filtered_cloud;
pcl::io::savePCDFileASCII("final_cloud.pcd",*filtered_cloud);
}

void projectPoint(double x,double y,double z,int *u,int *v )
{


Eigen::Vector4f worldCoord;
worldCoord<< x,	  y,	z,		1;
Eigen::Vector3f cameraCoord;
cameraCoord = projectionMat* worldCoord;
cameraCoord(0) = cameraCoord(0)/ cameraCoord(2);
cameraCoord(1) = cameraCoord(1)/ cameraCoord(2);
cameraCoord(2) = cameraCoord(2)/ cameraCoord(2);

Eigen::Vector3f projectedCoord;
projectedCoord = CI * cameraCoord;
//baseCoord = laserCoord;
//std::cout << baseCoord<<endl;
*u = (int)projectedCoord(0);
*v = (int)projectedCoord(1);
}

void projectCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , cv::Mat boat2D)
{

cout<<"Projecting cloud"<<endl;	
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr grayCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointXYZRGBNormal basic_point;
cout<<cloud->points.size()<<endl;
for(int m=2055 ; m>(-1);m--)
for(int n=2453; n>=(-1);n--)
backProjection[m][n] = (0);

for (int counter=0; counter< cloud->points.size() ;counter++)
{
/*Eigen::Matrix<float, 3, 4> projectionMat;

projectionMat<< 1.0000,   -0.0003,    0.0032, -0.1610736,
-0.0005,   -0.9980,    0.0627,  0.1409484,
0.0031,   -0.0627,   -0.9980,  0.4056451;*/
Eigen::Vector4f worldCoord;
worldCoord<<	cloud->points[counter].x,  cloud->points[counter].y,	cloud->points[counter].z,	1;

Eigen::Vector3f cameraCoord;
cameraCoord = projectionMat* worldCoord;
cameraCoord(0) = cameraCoord(0)/ cameraCoord(2);
cameraCoord(1) = cameraCoord(1)/ cameraCoord(2);
cameraCoord(2) = cameraCoord(2)/ cameraCoord(2);

			
Eigen::Vector3f projectedCoord;
projectedCoord = CI * cameraCoord;
//baseCoord = laserCoord;
//std::cout << baseCoord<<endl;

basic_point.x = cloud->points[counter].x;
basic_point.y = cloud->points[counter].y;
basic_point.z = cloud->points[counter].z;
basic_point.normal_x = cameraCoord(0);
basic_point.normal_y = cameraCoord(1);
basic_point.normal_z = cameraCoord(2);
//string ty =  type2str( boat2D.type() );
basic_point.r = (int) (boat2D.at<uchar> ((int)projectedCoord(1), (int)projectedCoord(0)));
basic_point.g = (int) (boat2D.at<uchar> ((int)projectedCoord(1), (int)projectedCoord(0)));
basic_point.b = (int) (boat2D.at<uchar> ((int)projectedCoord(1), (int)projectedCoord(0)));
backProjection[(int)projectedCoord(1)][(int)projectedCoord(0)] = counter;
grayCloud->points.push_back(basic_point);
			
//std::cout << basic_point<<endl;
}
	
cout<<"projected cloud"<<endl;
grayCloud->width = (int) grayCloud->points.size ();
grayCloud->height = 1;
cout<<*grayCloud<<endl;

//pcl::io::loadPCDFile ("my_point_cloud.pcd", basic_boat);	
//cout<<"writing cloud"<<endl;	
gray_boat = grayCloud;
//pcl::io::savePCDFileASCII("gray_cloud.pcd",*gray_boat);
/*pcl::PCDWriter writer;
writer.write ("my_point_cloud.pcd", *cloud);*/
//cout<<"writen cloud"<<endl;	
}
};
