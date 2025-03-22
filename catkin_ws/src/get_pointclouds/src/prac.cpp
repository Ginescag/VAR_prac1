	
#include <iostream>


using namespace std;


#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "prac1_node");
    ros::NodeHandle nh;
    ROS_INFO("¡Hola! Este es el nodo prac1.");
    ros::spin();
    return 0;
}
    //deteccion de caracteristicas con ISS
	// pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> detectorISS;
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);

	// detectorISS.setInputCloud(cloud_filtered);
	// detectorISS.setSalientRadius(0.2);
	// detectorISS.setNonMaxRadius(0.4);
	// detectorISS.setThreshold21(0.975);
	// detectorISS.setThreshold32(0.975);
	// detectorISS.setMinNeighbors(5);
	// detectorISS.setNumberOfThreads(4);
	// detectorISS.compute(*keypoints);

	// cout << "Puntos tras ISS: " << keypoints->size() << endl;

	// //usar RANSAC para emparejar las nubes de puntos


	// pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ransac;
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	// ransac.setInputSource(cloud_filtered);
	// ransac.setInputTarget(keypoints);
	// ransac.align(*aligned_cloud);

	// if(ransac.hasConverged())
	// {
	// 	cout << "ransac converged" << endl;
	// 	cout << "The score is: " << ransac.getFitnessScore() << endl;
	// 	cout << "Transformation matrix:" << endl;
	// 	cout << ransac.getFinalTransformation() << endl;

	// }
	// else
	// {
	// 	cout << "ransac did not converge" << endl;
	// }
