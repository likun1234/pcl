#include <iostream>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;


typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::Boundary Boundary;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointRGBT> RGBCloud;
typedef pcl::Normal Normal;

typedef pcl::PointXYZRGB RGBPointT;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::FPFHSignature33 Feature;
typedef pcl::FPFHEstimationOMP<PointT,Normal,Feature> FeatureEstimation;
typedef pcl::PointCloud<Feature> FeatureCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandler;





void statistcalFilter(PointCloud::Ptr cloud, PointCloud::Ptr output)
{
    pcl::StatisticalOutlierRemoval<PointT> sor; 
    sor.setInputCloud(cloud);     
    sor.setMeanK(200);     
    sor.setStddevMulThresh(1.0);     
    sor.filter(*output);     
}


void boundary_exract(PointCloud::Ptr cloud, PointCloud::Ptr boundPoints)
{
    if (0 == cloud->points.size())
    {
        cout<<"the point have no data"<<endl;
        exit(1);
    }
    std::cout << "points sieze is:"<< cloud->size()<<std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<Boundary> boundaries;
    pcl::BoundaryEstimation<PointT,Normal,Boundary> est;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());



    pcl::NormalEstimation<PointT,pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项>是曲率
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree);
    // normEst.setRadiusSearch(2);  //法向估计的半径
    normEst.setKSearch(9);  //法向估计的点数
    normEst.compute(*normals);
    cout<<"normal size is "<< normals->size()<<endl;

    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    est.setSearchMethod (tree);
    est.setKSearch(20);  //一般这里的数值越高，最终边界识别的精度越好
    est.compute (boundaries);

    //pcl::PointCloud<PointT>::Ptr boundPoints (new               pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointT> noBoundPoints;
    int countBoundaries = 0;
    for (int i=0; i<cloud->size(); i++){
        uint8_t x = (boundaries.points[i].boundary_point);
        int a = static_cast<int>(x); //该函数的功能是强制类型转换
        if ( a == 1)
        {
            ( *boundPoints).push_back(cloud->points[i]);
            countBoundaries++;
        }
        else
            noBoundPoints.push_back(cloud->points[i]);

    }
    std::cout<<"boudary size is：" <<countBoundaries <<std::endl;

    pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
    //pcl::io::savePCDFileASCII("NoBoundpoints.pcd",noBoundPoints);
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(boundPoints);
    while (!viewer.wasStopped())
    {
    }
}


int main(int argc, char **argv)
{
    PointCLoud::Ptr cloud (new PointCloud());
    PointCLoud::Ptr boundarypoint (new PointCloud());
    if (-1 == pcl::loadPCDFile(argv[1], *cloud))
    {
        cout<<"load fail"<<endl;
        exit(1);
	}
    
}


//int main(int argc, char **argv)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	// if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/yxg/pcl/pcd/mid.pcd",*cloud) == -1)
//	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1],*cloud) == -1)
//	{
//		PCL_ERROR("COULD NOT READ FILE mid.pcl \n");
//		return (-1);
//	}
//
//	std::cout << "points sieze is:"<< cloud->size()<<std::endl;
//	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//	pcl::PointCloud<pcl::Boundary> boundaries;
//	pcl::BoundaryEstimation<pcl::PointXYZ,pcl::Normal,pcl::Boundary> est;
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//	/*
//	   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
//	   kdtree.setInputCloud(cloud);
//	   int k =2;
//	   float everagedistance =0;
//	   for (int i =0; i < cloud->size()/2;i++)
//	   {
//	   vector<int> nnh ;
//	   vector<float> squaredistance;
//	//  pcl::PointXYZ p;
//	//   p = cloud->points[i];
//	kdtree.nearestKSearch(cloud->points[i],k,nnh,squaredistance);
//	everagedistance += sqrt(squaredistance[1]);
//	//   cout<<everagedistance<<endl;
//	}
//
//	everagedistance = everagedistance/(cloud->size()/2);
//	cout<<"everage distance is : "<<everagedistance<<endl;
//
//*/
//
//
//
//	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
//	normEst.setInputCloud(cloud);
//	normEst.setSearchMethod(tree);
//	// normEst.setRadiusSearch(2);  //法向估计的半径
//	normEst.setKSearch(9);  //法向估计的点数
//	normEst.compute(*normals);
//	cout<<"normal size is "<< normals->size()<<endl;
//
//	est.setInputCloud(cloud);
//	est.setInputNormals(normals);
//	est.setSearchMethod (tree);
//	est.setKSearch(20);  //一般这里的数值越高，最终边界识别的精度越好
//	est.compute (boundaries);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints (new               pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
//	int countBoundaries = 0;
//	for (int i=0; i<cloud->size(); i++){
//		uint8_t x = (boundaries.points[i].boundary_point);
//		int a = static_cast<int>(x); //该函数的功能是强制类型转换
//		if ( a == 1)
//		{
//			( *boundPoints).push_back(cloud->points[i]);
//			countBoundaries++;
//		}
//		else
//			noBoundPoints.push_back(cloud->points[i]);
//
//	}
//	std::cout<<"boudary size is：" <<countBoundaries <<std::endl;
//
//	pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);
//	pcl::io::savePCDFileASCII("NoBoundpoints.pcd",noBoundPoints);
//	pcl::visualization::CloudViewer viewer ("test");
//	viewer.showCloud(boundPoints);
//	while (!viewer.wasStopped())
//	{
//	}
//	return 0;
//}
