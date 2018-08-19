
#include <iostream>
#include <cstring>
#include <opencv2/opencv.hpp>
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


typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGB RGBPointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<RGBPointT> RGBCloud;


void color(cv::Mat img, PointCloud::Ptr cloud, RGBCloud::Ptr color_cloud)//根据彩图和点云生成彩色点云
{
	int width = img.cols;
	int height = img.rows;
	for (int i=0;i<width;i++)
		for (int j=0;j<height;j++)
		{
			double index = j * width + i;
			RGBPointT point;
			point.x =  cloud->points[index].x;
			point.y =  cloud->points[index].y;
			point.z =  cloud->points[index].z;
			point.b =  img.at<cv::Vec3b>(j,i)[0];
			point.g =  img.at<cv::Vec3b>(j,i)[1];
			point.r =  img.at<cv::Vec3b>(j,i)[2];
			color_cloud->push_back(point);
		}
	color_cloud->height = 1;
	color_cloud->width = color_cloud->points.size();
}


int
main(int argc,char **argv)
{
	cv::Mat img = cv::imread(argv[1]);
	PointCloud::Ptr cloud (new PointCloud());
	RGBCloud::Ptr rgbcloud (new RGBCloud());
	if (-1 == pcl::io::loadPCDFile(argv[2], *cloud)){
		std::cout<<"pcd load fail"<<std::endl; 
		exit(0);
	}
	color(img, cloud, rgbcloud);
	pcl::io::savePCDFile("rgbcloud.pcd", *rgbcloud);
}
