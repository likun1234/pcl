#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>     
#include <pcl/correspondence.h>   
#include <pcl/features/normal_3d_omp.h> 
#include <pcl/features/shot_omp.h>   
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>  
#include <pcl/recognition/cg/hough_3d.h>   
#include <pcl/recognition/cg/geometric_consistency.h> 
#include <pcl/visualization/pcl_visualizer.h>   
#include <pcl/kdtree/kdtree_flann.h>            
#include <pcl/kdtree/impl/kdtree_flann.hpp>   
#include <pcl/common/transforms.h>             
#include <pcl/console/parse.h>
#include <pcl/filters/morphological_filter.h>
//#include <pcl/filters/impl/morphological_filter.hpp>
using namespace pcl;
using namespace std;
void color_gray(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
    for(int i = 0; i<cloud_color->points.size(); i++)
    {
        pcl::PointXYZ point;
        point.x = cloud_color->points[i].x;
        point.y = cloud_color->points[i].y;
        point.z = cloud_color->points[i].z;
        cloud->push_back(point);
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
}
int main(int argc,char **argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(-1 == pcl::io::loadPCDFile(argv[1], *cloud_color))
    {
        cout<<"load point fail"<<endl;
    }
    color_gray(cloud_color, cloud);
    pcl::io::savePCDFile("gray.pcd", *cloud);
}
