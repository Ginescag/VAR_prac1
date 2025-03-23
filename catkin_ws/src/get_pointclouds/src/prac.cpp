	
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/gicp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "sub_prac");
	for (int i = 0; i < 16; i++){
		std::string filename = "cloud_filtered" + std::to_string(i) +".pcd";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud_filtered) == -1) {
			PCL_ERROR ("Couldn't read file cloud_filtered.pcd \n");
			return (-1);
		}
		std::cout << "Puntos en la nube original: " << cloud_filtered->size() << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		// pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss;
		// iss.setInputCloud(cloud_filtered);
		// iss.setSalientRadius(0.2);
		// iss.setNonMaxRadius(0.4);
		// iss.setThreshold21(0.975);
		// iss.setThreshold32(0.975);
		// iss.setMinNeighbors(5);
		// iss.setNumberOfThreads(4);
		// iss.compute(*keypoints);

		pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB> sift;
		sift.setInputCloud(cloud_filtered);
		sift.setScales(0.005f, 5, 4);
		sift.setMinimumContrast(0.005f);
		sift.compute(*keypoints);

		// pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_harris(new pcl::PointCloud<pcl::PointXYZI>); 

		// pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> harris;
		// harris.setRadius(0.05f);       // Radio de búsqueda (5 cm)
		// harris.setThreshold(0.001f);   // Umbral bajo
		// harris.setNonMaxSupression(true);
		// harris.compute(*keypoints_harris);



		std::cout << "Puntos claves detectados en "+filename+": " << keypoints->size() << std::endl;
	}

}
