//C++
#include <iostream>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

//boost
#include <boost/thread/thread.hpp>

//OpenCV
#include <opencv2/opencv.hpp>

//Eigen
#include <Eigen/Eigen>

//Customed
#include "config.h"
#include "pointcloud_helper.h"
#include "transformformat.hpp"

using namespace std;
using namespace pcl;
using namespace cv;
using namespace Eigen;


#ifndef PI
#define PI 3.1415926535897932384626433832795028841971693993751058209749445
#endif
#ifndef pi
#define pi 3.1415926535897932384626433832795028841971693993751058209749445
#endif

// #define ColorPoints
// #define PointTypeHere pcl::PointXYZRGB

#define Points
#define PointTypeHere pcl::PointXYZ

int main (int argc, char** argv)
{
	std::vector<pcl::PointCloud<PointTypeHere>::Ptr > pc_source_ptr;
	std::vector<Eigen::Matrix4d> Mat_base2tool;
	std::vector<std::string> pc_filename;

	// //创建点云对象
	// pcl::PointCloud<PointTypeHere>::Ptr cloud(new pcl::PointCloud<PointTypeHere>);

	// //加载点云数据，命令行参数为文件名，文件可以是.PCD/.pcd/.PLY/.ply四种文件之一，单位可以是mm也可以是m.
	// if(loadPointCloudData(pc_filename[0], cloud) != 0)
	// 	exit(-1);



	pcl::PointCloud<PointTypeHere>::Ptr target_cloud (new pcl::PointCloud<PointTypeHere>);
	if (pcl::io::loadPCDFile<PointTypeHere> (argv[1], *target_cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
	std::cout << "Loaded " << target_cloud->size () << " data points from " << argv[1] << std::endl;
	pc_source_ptr.push_back(target_cloud);


	pcl::PointCloud<PointTypeHere>::Ptr input_cloud (new pcl::PointCloud<PointTypeHere>);
	if (pcl::io::loadPCDFile<PointTypeHere> (argv[2], *input_cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
	std::cout << "Loaded " << input_cloud->size () << " data points from " << argv[2] << std::endl;
	pc_source_ptr.push_back(input_cloud);

	pcl::PointCloud<PointTypeHere>::Ptr filtered_cloud (new pcl::PointCloud<PointTypeHere>);
	pcl::ApproximateVoxelGrid<PointTypeHere> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud (input_cloud);
	approximate_voxel_filter.filter (*filtered_cloud);
	std::cout << "Filtered cloud contains " << filtered_cloud->size ()
	<< " data points from room_scan2.pcd" << std::endl;


	Eigen::Vector3d trans_vector;
	Eigen::Vector3d RPYdeg_vector;

	ifstream ifs;
	ifs.open(argv[3], ios::in);

	ifs >> trans_vector(0) >> trans_vector(1) >> trans_vector(2);
	ifs >> RPYdeg_vector(0) >> RPYdeg_vector(1) >> RPYdeg_vector(2);
	Eigen::Matrix4d Mbt1 = Vector3ds2Matrix4d_RPY(trans_vector, 0.001, RPYdeg_vector, CV_PI/180.0);
	std::cout << Mbt1 <<std::endl;

	ifs >> trans_vector(0) >> trans_vector(1) >> trans_vector(2);
	ifs >> RPYdeg_vector(0) >> RPYdeg_vector(1) >> RPYdeg_vector(2);
	Eigen::Matrix4d Mbt2 = Vector3ds2Matrix4d_RPY(trans_vector, 0.001, RPYdeg_vector, CV_PI/180.0);
	std::cout << Mbt2 <<std::endl;

	ifs.close();

	ifs.open(argv[4], ios::in);
	ifs >> trans_vector(0) >> trans_vector(1) >> trans_vector(2);
	ifs >> RPYdeg_vector(0) >> RPYdeg_vector(1) >> RPYdeg_vector(2);
	Eigen::Matrix4d Meh = Vector3ds2Matrix4d_AngleAxis(trans_vector, 0.001, RPYdeg_vector, CV_PI/180.0);
	std::cout << Meh <<std::endl;
	ifs.close();


	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	viewer1 (new pcl::visualization::PCLVisualizer ("Viewer"));
	viewer1->setBackgroundColor (0, 0, 0);


	pcl::PointCloud<PointTypeHere>::Ptr output_cloud1 (new pcl::PointCloud<PointTypeHere>);
	pcl::transformPointCloud (*pc_source_ptr[0], *output_cloud1, Mbt1.cast<float>()*Meh.cast<float>());

	pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>	target_cloud_color (target_cloud, 255, 0, 0);
	viewer1->addPointCloud<PointTypeHere> (target_cloud, target_cloud_color, "target_cloud");
	viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

	// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(target_cloud);
	// viewer1->addPointCloud<pcl::PointXYZRGB> (target_cloud, rgb, "target_cloud");
	// viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_cloud");


	pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>	output_cloud1_color (output_cloud1, 255, 0, 0);
	viewer1->addPointCloud<PointTypeHere> (output_cloud1, output_cloud1_color, "output_cloud1");
	viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud1");






	pcl::PointCloud<PointTypeHere>::Ptr output_cloud2 (new pcl::PointCloud<PointTypeHere>);
	pcl::transformPointCloud (*input_cloud, *output_cloud2, Mbt2.cast<float>()*Meh.cast<float>());

	pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>	input_cloud_color (input_cloud, 0, 255, 0);
	viewer1->addPointCloud<PointTypeHere> (input_cloud, input_cloud_color, "input_cloud");
	viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>	output_cloud2_color (output_cloud2, 0, 0, 255);
	viewer1->addPointCloud<PointTypeHere> (output_cloud2, output_cloud2_color, "output_cloud2");
	viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud2");

	while (!viewer1->wasStopped ())
	{
		viewer1->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}




	ifs.close();

	Eigen::Matrix4d MatIdentity;
	MatIdentity.setIdentity();

	double start = (double)(cv::getTickCount());
	pcl::IterativeClosestPoint<PointTypeHere, PointTypeHere> icp;
	icp.setMaximumIterations (1000);
	icp.setInputSource(filtered_cloud);
	icp.setInputTarget(target_cloud);
	pcl::PointCloud<PointTypeHere> Final;
	// icp.align (Final, (Mbt1.inverse()*Mbt2).cast<float>());
	icp.align (Final, MatIdentity.cast<float>());
	std::cout << icp.getFinalTransformation() << std::endl;
	std::cout << "init  pose:" << (Mbt1.inverse()*Mbt2).cast<float>() << std::endl;
	std::cout << "final pose:" << icp.getFinalTransformation() << std::endl;
	double end = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
	std::cout << "所用时间" << end <<std::endl;

	pcl::PointCloud<PointTypeHere>::Ptr output_cloud (new pcl::PointCloud<PointTypeHere>);
	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*input_cloud, *output_cloud, icp.getFinalTransformation ());
	// pcl::transformPointCloud (*input_cloud, *output_cloud, MatIdentity);




	// // Saving transformed input cloud.
	// // pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

 //  // Initializing point cloud visualizer
	// boost::shared_ptr<pcl::visualization::PCLVisualizer>
	// viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer_final->setBackgroundColor (0, 0, 0);

 //  // Coloring and visualizing target cloud (red).
	// pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>
	// target_color (target_cloud, 255, 0, 0);
	// viewer_final->addPointCloud<PointTypeHere> (target_cloud, target_color, "target cloud");
	// viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
	// 	1, "target cloud");

 //  // Coloring and visualizing transformed input cloud (green).
	// pcl::visualization::PointCloudColorHandlerCustom<PointTypeHere>
	// output_color (output_cloud, 0, 255, 0);
	// viewer_final->addPointCloud<PointTypeHere> (output_cloud, output_color, "output cloud");
	// viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
	// 	1, "output cloud");

 //  // Starting visualizer
	// viewer_final->addCoordinateSystem (1.0, 0 );
 //  // viewer_final->addCoordinateSystem (1.0, "global");
	// viewer_final->initCameraParameters ();

 //  // Wait until visualizer window is closed.
	// while (!viewer_final->wasStopped ())
	// {
	// 	viewer_final->spinOnce (100);
	// 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	// }



	return (0);
}
