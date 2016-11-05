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
	double cloudArr[1000][1280][8];
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_boat ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr resampled_boat;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_boat;  
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr gray_boat;
	Eigen::Matrix<float, 3, 4> projectionMat;
	Eigen::Matrix3f CI;
	int backProjection[2056][2454];
	//World Coordinates

	double lim_right;
	double lim_left;
	double lim_front;
	double lim_back;
	double lim_top;
	double lim_bottom; 			

	PCLdata()
	{                                
		//Initializing Parameters
		basic_boat = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		resampled_boat = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr ;
		filtered_boat = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>); 
		gray_boat = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		/*projectionMat<< 1.0000,   -0.0003,    0.0032, -0.1610736,
				-0.0005,   -0.9980,    0.0627,  0.1409484,
				0.0031,   -0.0627,   -0.9980,  0.4056451;*/
		/*CI<< 2297.44818700000,	0,	1089.54543100000,
					 0,	2291.86411800000,	935.870470000000,
					 0,	0,	1;*/

		/*projectionMat<< 0.9963,   -0.0052,    0.0861, -0.1934687,
						-0.0109,   -0.9978,    0.0656,  0.1296488,
						0.0855,   -0.0663,   -0.9941,  0.3904841;*/
		
		/*CI<< 2354.862624, 0.000000, 1285.167079,
			 0.000000, 2355.274166, 1050.767930,
			 0.000000, 0.000000, 1;*/

		projectionMat<<  -0.9966,    0.0214,   -0.0797,  0.5487686,
						0.0233,    0.9995,   -0.0232,  -0.0858538,
						0.0792,   -0.0249,   -0.9965,  0.2873345;
		
		CI<< 2176.211068, 0.000000, 1244.289249, 0.000000, 2178.633804, 1041.836938, 0.000000, 0.000000, 1.000000;

		lim_left = 435;//420;//140;
		lim_right= 660;//670;//365;
		lim_front=145;//150//194
		lim_back = 15;//10//5;//60
		lim_top= 87;
		lim_bottom = 14;

	}

void createCloud( int ctr, double (*arr)[1000][1280][8])
{
			////Starting segmentation

		////read file
		////scale, desample and filter data
		  //// ------------------------------------
  //// -----Create example point cloud-----
  //// ------------------------------------
  //pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //std::cout << "Genarating example point clouds.\n\n";
  //// We're going to make an ellipse extruded along the z-axis. The colour for
  //// the XYZRGB cloud will gradually go from red to green to blue.
  //uint8_t r(255), g(15), b(15);
  //for (float z(-1.0); z <= 1.0; z += 0.05)
  //{
  //  for (float angle(0.0); angle <= 360.0; angle += 5.0)
  //  {
  //    pcl::PointXYZ basic_point;
  //    basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
  //    basic_point.y = sinf (pcl::deg2rad(angle));
  //    basic_point.z = z;
  //    basic_cloud_ptr->points.push_back(basic_point);

  //    pcl::PointXYZRGB point;
  //    point.x = basic_point.x;
  //    point.y = basic_point.y;
  //    point.z = basic_point.z;
  //    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
  //            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  //    point.rgb = *reinterpret_cast<float*>(&rgb);
  //    point_cloud_ptr->points.push_back (point);
  //  }
  //  if (z < 0.0)
  //  {
  //    r -= 12;
  //    g += 12;
  //  }
  //  else
  //  {
  //    g -= 12;
  //    b += 12;
  //  }
  //}

	//Base to tool using matrix roatation on x,y,z,a,b,c
	//Then laser to tool using static matrix...
	//finally laser coordinate in base frame  =  B_T_T X T_T_L X [0 Y Z 1]
	


//	tx=182.96;%182.45;%3 pellet182.19;%92.50;
//ty=200.70;%3 pellet 212.37;%237.48;
//tz=158.80;%3 pellet 157.25;
//a=-53.98;
//b=-1.13;
//c=179.92;
//
//
//tend=252.92;
//
//res=((tend-tx)/54);
//
//
//%   calc of R|T matrix
//Rx=[1 0 0;0 cosd(c) -sind(c);0 sind(c) cosd(c)];
//Ry=[cosd(b) 0 sind(b); 0 1 0;-sind(b) 0 cosd(b)];
//Rz=[cosd(a) -sind(a) 0;sind(a) cosd(a) 0;0 0 1];
//Q=Rz*Ry*Rx;
//T=[tx;ty;tz];
//B_T_T=[Q T;0 0 0 1];%Base with respt to tool matrix
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//CI=[2297.448187 0.000000 1089.545431; 0.000000 2291.864118 935.870470; 0.000000 0.000000 1];     %Internal Paramter
//ROT_Y = [cos(beta1),0,sin(beta1);  0,1,0 ; -sin(beta1),0,cos(beta1)];
//ROT_X = [ 1,0,0 ; 0,cos(gamma1),-sin(gamma1);  0,sin(gamma1),cos(gamma1)];
//ROT_Z = [ cos(alpha1),-sin(alpha1),0; sin(alpha1),cos(alpha1),0 ; 0,0,1];
//R_KUKA = ROT_Z*ROT_Y*ROT_X;
//t_new=[t_x1,t_y1,t_z1];
//C_T_L=[R_KUKA,t_new';0 0 0 1];
//T_T_L=[-0.8232 -0.5674 -0.0202 114.0241;-0.5665  0.8233 -0.0370 45.605; 0.0377 -0.0190 -0.9991 340.2798; 0 0 0 1];

	/*	Matrix3f m;
m << 1, 2, 3,
     4, 5, 6,
     7, 8, 9;
std::cout << m;*/
//[ cos(a)*cos(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b)]
//[ cos(b)*sin(a), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c)]
//[       -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c)]


//float x_L = (*arr)[1000][1280][0],z_L = (*arr)[1000][1280][1],x = (*arr)[1000][1280][2],y = (*arr)[1000][1280][3],z = (*arr)[1000][1280][4],a = (*arr)[1000][1280][5],b = (*arr)[1000][1280][6],c = (*arr)[1000][1280][7];
//
//Eigen::Matrix4f B_T_T;
//B_T_T << cos(a)*cos(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b), x,
//		 cos(b)*sin(a), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c), y,
//			   -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c), z,
//					 0,										0,									0,  1;
//std::cout << B_T_T<<endl;
//
//Eigen::Matrix4f T_T_L;
//T_T_L<<-0.8232, -0.5674, -0.0202, 114.0241,
//	   -0.5665,  0.8233, -0.0370, 45.605,
//	    0.0377, -0.0190, -0.9991, 340.2798,
//			 0,		  0,	   0,		1;
//std::cout << T_T_L<<endl;
//
//Eigen::Vector4f laserCoord;
//laserCoord<<		x_L,	  0,	z_L,		1;
//std::cout << laserCoord<<endl;
//
//Eigen::Vector4f baseCoord;
//baseCoord = B_T_T * T_T_L * laserCoord;

		
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
	for (int profile_no=0;profile_no<ctr;profile_no++)
		{
			unsigned int i=0;
			float x = (*arr)[profile_no][i][2],y = (*arr)[profile_no][i][3],z = (*arr)[profile_no][i][4],a = (*arr)[profile_no][i][5]* PI / 180.0,b = (*arr)[profile_no][i][6]* PI / 180.0,c = (*arr)[profile_no][i][7]* PI / 180.0;
				
				Eigen::Matrix4f B_T_T;
				B_T_T << cos(a)*cos(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b), x ,
						 cos(b)*sin(a), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c), y ,
							   -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c), z ,
									 0,									0,										0,	1;

				//std::cout << B_T_T<<endl;
				
				Eigen::Matrix4f T_T_L;
				/*T_T_L<<-0.8232, -0.5674, -0.0202, 114.0241,
					   -0.5665,  0.8233, -0.0370, 45.605,
					    0.0377, -0.0190, -0.9991, 340.2798,
							 0,		  0,	   0,		1;*/
				/*T_T_L<<-0.7986,   -0.6007,   -0.0370,  113.9513,
						-0.6004,    0.7995,   -0.0201,   45.4287,
						 0.0416,    0.0061,   -0.9991,  338.8648,
						 0,         0,         0,    1.0000;*/
				T_T_L<<        -0.7872,   -0.6154,   -0.0402,  115.7029,
								-0.6158,    0.7879,   -0.0024,   41.0518,
								0.0332,    0.0228,   -0.9992,  328.6233,
									0 ,        0   ,      0  ,  1.0000;
				//std::cout << T_T_L<<endl;

				Eigen::Matrix4f B_T_L;
				B_T_L = B_T_T * T_T_L ;
				//B_T_L = T_T_L;
				//std::cout << T_T_L<<endl;

			for(i=0; i<1280; i++)
			{
				float  x_L = (*arr)[profile_no][i][0],z_L = (*arr)[profile_no][i][1];
				
				Eigen::Vector4f laserCoord;
				laserCoord<<		x_L,	  0,	z_L,		1;
				//std::cout << laserCoord<<endl;
								
				Eigen::Vector4f baseCoord;
				baseCoord = B_T_L* laserCoord;
				//baseCoord = laserCoord;
				//std::cout << baseCoord<<endl;

				basic_point.x = baseCoord[0]-1;
				basic_point.y = baseCoord[1]+1;
				basic_point.z = baseCoord[2]-4;
				cloud->points.push_back(basic_point);
			}
			//std::cout << basic_point<<endl;
		}
		
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

void scale()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
  //pcl::PCDReader reader;
  //cout<<"reading cloud"<<endl;	
  // Replace the path below with the path where you saved your file
 // reader.read ("my_point_cloud.pcd", *cloud); // Remember to download the file first!
	//pcl::io::loadPCDFile<pcl::PointXYZ> ("my_point_cloud_trans.pcd", *cloud);
  //cloud = basic_boat;
	//Resize Image
  //cout<<"read cloud"<<endl;	
	std::cout<<" rescaling\n";
	std::cout << "Cloud before rescaling: " << std::endl;
  std::cout << *basic_boat << std::endl;
  pcl::PointXYZ basic_point;
  		lim_left = 435;//420;//140;
		lim_right= 660;//670;//365;
		lim_front=145;//150//194
		lim_back = 15;//10//5;//60
		lim_top= 87;
		lim_bottom = 14;
	for(int i=0;i<basic_boat->size();i++)
			{
				if(!(((basic_boat->points[i].x)>lim_right) || ((basic_boat->points[i].x)<lim_left) || ((basic_boat->points[i].y)>lim_front) || ((basic_boat->points[i].y)<lim_back) || ((basic_boat->points[i].z)>87) || ((basic_boat->points[i].z)<14)))
				{
					basic_point.x=basic_boat->points[i].x/1000.0;
					basic_point.y=basic_boat->points[i].y/1000.0;
					basic_point.z=  basic_boat->points[i].z/1000.0;
					cloud->points.push_back(basic_point);
				}
				//else 
				//{
					/*basic_point.x=basic_boat->points[i].x/1000.0;
					basic_point.y=basic_boat->points[i].y/1000.0;
					basic_point.z=  basic_boat->points[i].z/1000.0;
					cloud->points.push_back(basic_point);*/
				//}
			}
	 std::cout<<"done rescaling\n";
	 std::cout << "Cloud after rescaling: " << std::endl;
	 cloud->width = (int) cloud->points.size ();
	cloud->height = 1;
	std::cout << *cloud << std::endl;
	 //pcl::io::savePCDFileASCII("test_pcd.pcd",*cloud);
	 basic_boat = cloud;
}

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
  std::cout << "PointCloud before Desampling: " << cloud_2->width * cloud_2->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";
  //cout<<"hello"<<endl;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_2);
  sor.setLeafSize (0.001f, 0.001f, 0.001f);
  sor.filter (*resampled_cloud_2);

  //fromPCLPointCloud2( *cloud_2, *cloud);
  fromPCLPointCloud2(*resampled_cloud_2, *resampled_cloud);

  std::cout<< "PointCloud after Desampling: " << resampled_cloud->width * resampled_cloud->height 
       << " data points (" << pcl::getFieldsList (*resampled_cloud) << ").";
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
  #if _DEBUG
  pcl::io::savePCDFileASCII("final_cloud.pcd",*filtered_cloud);
#endif
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

				/*Eigen::Matrix3f CI;
				CI<< 2297.44818700000,	0,	1089.54543100000,
					 0,	2291.86411800000,	935.870470000000,
					 0,	0,	1;*/

				/*Eigen::Matrix<float, 3, 4> cameraMatrix;

				cameraMatrix = CI * projectionMat;*/

				
				//std::cout << laserCoord<<endl;
								
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