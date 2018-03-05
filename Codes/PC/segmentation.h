#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// filtering
#include <pcl/filters/statistical_outlier_removal.h>

//regionGrowing
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
//Using PCL visualizer so don't need
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

//PCE principal curvatur estimation
#include <pcl/features/principal_curvatures.h>

//Moment of Inertia estimation
#include <pcl/features/moment_of_inertia_estimation.h>


