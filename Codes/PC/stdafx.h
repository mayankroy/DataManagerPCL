// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#include <stdlib.h>  
#include <pcl/common/common_headers.h>
#include <stdio.h>
#include <tchar.h>
#include<conio.h>
#include<io.h>
#include <iostream>
#include <math.h> 
#include<string.h>
#include<highgui.h>
#include<cv.h>
#include <vector>
#include <list>
#include <map>
#include <deque>
#include <sstream>
#include <fstream>
#include <ostream>
#include <istream>
#include <iomanip>
#include <algorithm>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

//#include <pylon/PylonIncludes.h>
//#include <pylon/gige/BaslerGigECamera.h>
//
//using namespace Pylon;
//using namespace Basler_GigECameraParams;
//
//typedef struct 
//	{
//		CvPoint3D32f position;
//		float orientation;
//	}pos_orient;
//
//
////!!!!Z is always relative!!!!
//typedef struct 
//	{
//		CvPoint3D32f position;
//		float orientation;
//		bool gripper;
//	}pos_orient_gripper;
//
//typedef struct 
//	{
//		CvPoint3D32f position;
//		float orientation;
//		bool standing;
//	}pos_orient_standing;
//	
//typedef struct 
//	{
//		CvPoint3D32f position;
//		float orientation;
//		bool gripper;
//		bool standing;
//	}pos_orient_gripper_standing;
//
//
//	
//#define longest_edge  120//190 //90
//#define shortest_edge 100 //50
//
//typedef CBaslerGigECamera Camera_t;
//
//
//// TODO: reference additional headers your program requires here
