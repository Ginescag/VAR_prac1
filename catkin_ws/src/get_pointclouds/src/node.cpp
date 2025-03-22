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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;


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
		pcl::io::savePCDFileASCII("aligned_cloud.pcd", *aligned_cloud);
        std::cout << "La nube de puntos alineada ha sido guardada en 'aligned_cloud.pcd'" << std::endl;
	}
	else
	{
		cout << "ransac did not converge" << endl;
	}


	visu_pc = aligned_cloud;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

  boost::thread t(simpleVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }

}
