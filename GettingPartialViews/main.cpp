#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <vtkPolyDataMapper.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <iostream>
#include <stdio.h>
#include <pcl/conversions.h>
#include <cstring>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/centroid.h>
#include <Eigen/Core>
#include <cstdlib>


#include <pcl/common/time.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/bilateral.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<pcl::Normal> NormalCloud;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud<PointRGBT> RGBCloud;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandler;
int getAllSurfaces(PointCloud::Ptr cloud, std::vector<PointCloud::Ptr> &outputList){
	float max_distance = 1;     //5-6
	size_t min_percentage = 25;
	size_t pointsSizeThresh = 1000;
	// Get segmentation ready 
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<PointT> seg;
	pcl::ExtractIndices<PointT> extract;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (max_distance);

	// Create pointcloud to publish inliers
	size_t original_size(cloud->height * cloud->width);
	int n_planes(0);
	while (cloud->height*cloud->width > original_size*min_percentage / 100){
		PointCloud::Ptr output(new PointCloud());
		// Fit a plane
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);

		// Check result
		if (inliers->indices.size() == 0)
			break;

		// Compute Standard deviation
		if (inliers->indices.size() > pointsSizeThresh) {
			output->is_dense = false; 
			output->width = 1; 
			output->height = inliers->indices.size();
			//output->points.resize(inliers->indices.size());
			for (size_t i=0;i<inliers->indices.size();i++){
				// Get Point
				PointT pt = cloud->points[inliers->indices[i]];
				output->points.push_back(pt);

			}
			outputList.push_back(output);
			std::string number_str = std::to_string(n_planes);
			pcl::io::savePCDFile("parts_" + number_str + ".pcd", *output);
			n_planes++;
		}

		// Extract inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		PointCloud cloudF;
		extract.filter(cloudF);
		cloud->swap(cloudF);

		// Nest iteration
		//save output
	}

	return n_planes;
}


int voxel_grid_filter (PointCloud::Ptr inputCloud, PointCloud::Ptr outputCloud)
{
	std::cerr << "PointCloud before filtering: " << inputCloud->size() << " data points." << std::endl;

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (inputCloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*outputCloud);

	std::cerr << "PointCloud after filtering: "<< outputCloud->size() <<  " data points." << std::endl;

	return (0);
}


void Downsampling(PointCloud::Ptr model, PointCloud::Ptr model_keypoints)
{
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud (model);
	uniform_sampling.setRadiusSearch (0.01);
	uniform_sampling.filter (*model_keypoints);
	std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
}


void ply_pcd(const char* filepath)
{
	// Load the PLY model from a file.
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(filepath);
	reader->Update();
	// VTK is not exactly straightforward...
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	mapper->Update();

	vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

	// Virtual scanner object.
	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.addModelFromPolyData(object);
	// Pixel width of the rendering window, it directly affects the snapshot file size.
	render_views.setResolution(150);
	// Horizontal FoV of the virtual camera.
	render_views.setViewAngle(60.0f);
	// If true, the resulting clouds of the snapshots will be organized.
	render_views.setGenOrganized(true);
	// How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
	render_views.setTesselationLevel(1);
	// If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
	// This will affect the number of snapshots produced (if true, less will be made).
	// True: 42 for level 1, 162 for level 2, 642 for level 3...
	// False: 80 for level 1, 320 for level 2, 1280 for level 3...
	render_views.setUseVertices(true);
	// If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
	render_views.setComputeEntropies(true);

	render_views.generateViews();

	// Object for storing the rendered views.
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
	// Object for storing the poses, as 4x4 transformation matrices.
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
	// Object for storing the entropies (optional).
	std::vector<float> entropies;
	render_views.getViews(views);
	render_views.getPoses(poses);
	render_views.getEntropies(entropies);

	for(int i = 0; i < views.size(); ++i)
	{
		//pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);
		//voxel_grid_filter(views[i], outputCloud);

		//pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
		//Downsampling(views[i], outputCloud);

		std::string PCDPath = "../modelPCDFiles/";
		char filename[15];
		sprintf(filename, "view%d.pcd", i);
		PCDPath += filename;
		pcl::io::savePCDFileASCII(PCDPath, *views[i]);
		//pcl::io::savePCDFileASCII(PCDPath, *outputCloud);
	}

}

void NormalEst(PointCloud::Ptr cloud, NormalCloud::Ptr normals)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
	norm_est.setNumberOfThreads(16);
	norm_est.setKSearch (10);
	norm_est.setInputCloud (cloud);
	norm_est.compute (*normals);
}

void add_point(PointCloud::Ptr cloud,  PointT point)
{
	std::cout<<"1  size is  "<<cloud->points.size()<<std::endl;
	cloud->points.push_back(point);
	cloud->height = 1;
	cloud->width = cloud->points.size();
	std::cout<<"2  size is  "<<cloud->points.size()<<std::endl;
}



void vis_point(PointCloud::Ptr usedScene, std::vector<double> centroid)
{
	PointT point_v;
	pcl::Normal point_normal;
	pcl::PointCloud<pcl::Normal>::Ptr modelnormal(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pcl::Normal>::Ptr grab_nl_ (new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<PointT>::Ptr grab_pt_ (new pcl::PointCloud<PointT>());
	point_v.x = centroid[0];
	point_v.y = centroid[1];
	point_v.z = centroid[2];
	grab_pt_->points.push_back(point_v);

	add_point(usedScene, point_v);
	NormalEst(usedScene,modelnormal);
	int number = usedScene->points.size();
	point_normal.normal_x = modelnormal->points[number-1].normal_x;
	point_normal.normal_y = modelnormal->points[number-1].normal_y;
	point_normal.normal_z = modelnormal->points[number-1].normal_z;
	grab_nl_->points.push_back(point_normal);

	pcl::visualization::PCLVisualizer visu("Alignment");
	visu.addPointCloud (usedScene, ColorHandler (usedScene, 0.0, 255.0, 0.0), "scene");
	visu.addPointCloudNormals<PointT, pcl::Normal> (grab_pt_, grab_nl_, 1, 15, "normals");

	visu.spin ();

}




void point_normal(PointCloud::Ptr model, PointCloud::Ptr grab_pt_, NormalCloud::Ptr grab_nl_ )//返回模型的抓取点和法向量
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*model, centroid);



	PointT point_v;
	pcl::Normal point_normal;
	pcl::PointCloud<pcl::Normal>::Ptr modelnormal(new pcl::PointCloud<pcl::Normal>());
	point_v.x = centroid[0];
	point_v.y = centroid[1];
	point_v.z = centroid[2];
	grab_pt_->points.push_back(point_v);

	add_point(model, point_v);
	NormalEst(model,modelnormal);
	int number = model->points.size();
	point_normal.normal_x = modelnormal->points[number-1].normal_x;
	point_normal.normal_y = modelnormal->points[number-1].normal_y;
	point_normal.normal_z = modelnormal->points[number-1].normal_z;
	grab_nl_->points.push_back(point_normal);

	pcl::visualization::PCLVisualizer visu("Alignment");
	visu.addPointCloud (model, ColorHandler (model, 0.0, 255.0, 0.0), "scene");
	visu.addPointCloudNormals<PointT, pcl::Normal> (grab_pt_, grab_nl_, 1, 15, "normals");

	visu.spin (); 

}





	int
main(int argc, char** argv)
{

	PointCloud::Ptr cloud_model (new PointCloud);
	PointCloud::Ptr model (new PointCloud);
	string filename = argv[1];
	if(-1==pcl::io::loadPCDFile(filename, *cloud_model))
	{
		std::cout<<"pcd is not loaded"<<std::endl;
		exit(1);
	}

	PointCloud::Ptr grab_point (new PointCloud);
	NormalCloud::Ptr grab_normal (new NormalCloud); 
	//point_normal(cloud_model,grab_point, grab_normal);

	std::vector<PointCloud::Ptr> outputList;
	getAllSurfaces(cloud_model, outputList);
	point_normal(outputList[0], grab_point, grab_normal);
}

