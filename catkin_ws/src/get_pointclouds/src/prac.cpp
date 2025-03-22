	
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/gicp.h>

using namespace std;


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_filtered.pcd", *cloud_filtered) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file cloud_filtered.pcd \n");
        return (-1);
    }
    //deteccion de caracteristicas con ISS
	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> detectorISS;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	detectorISS.setInputCloud(cloud_filtered);
	detectorISS.setSalientRadius(0.2);
	detectorISS.setNonMaxRadius(0.4);
	detectorISS.setThreshold21(0.975);
	detectorISS.setThreshold32(0.975);
	detectorISS.setMinNeighbors(5);
	detectorISS.setNumberOfThreads(4);
	detectorISS.compute(*keypoints);

	cout << "Puntos tras ISS: " << keypoints->size() << endl;

	//usar RANSAC para emparejar las nubes de puntos

	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ransac;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	ransac.setInputSource(cloud_filtered);
	ransac.setInputTarget(keypoints);
	ransac.align(*aligned_cloud);

	if(ransac.hasConverged())
	{
		cout << "ransac converged" << endl;
		cout << "The score is: " << ransac.getFitnessScore() << endl;
		cout << "Transformation matrix:" << endl;
		cout << ransac.getFinalTransformation() << endl;



	}
	else
	{
		cout << "ransac did not converge" << endl;
	}


    // 4. Visualizar la nube de puntos transformada
    pcl::visualization::CloudViewer viewer("Aligned Cloud Viewer");
    viewer.showCloud(aligned_cloud);

    while (!viewer.wasStopped()) {
        // Mantener la visualización abierta
    }

}
