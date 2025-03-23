	
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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>



int main(int argc, char** argv) {
	ros::init(argc, argv, "sub_prac");

	Eigen::Matrix4f TT = Eigen::Matrix4f::Identity();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);




	for (int i = 0; i < 16; i++){
		std::string filename = "cloud_filtered" + std::to_string(i) +".pcd";

		if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud_filtered) == -1) {
			PCL_ERROR ("Couldn't read file cloud_filtered.pcd \n");
			return (-1);
		}
		std::cout << "Puntos en la nube original: " << cloud_filtered->size() << std::endl;

		//AQUI LA LOGICA PARA EXTRAER KEYPOINTS

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

		// 4. Realizar la alineación de nubes de puntos utilizando RANSAC
        if (i > 0) {
            // Establecer el modelo de RANSAC (utilizamos un modelo de plano como ejemplo)
            pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(prev_cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
            // Configurar el RANSAC
        // Método RANSAC
            ransac.setMaxIterations(1000);                // Número máximo de iteraciones
            ransac.setDistanceThreshold(0.01);            // Distancia máxima permitida entre puntos

            // Ejecutar RANSAC para encontrar la mejor transformación
            if (ransac.computeModel() == 0) {
                std::cerr << "RANSAC no pudo encontrar una buena transformación." << std::endl;
                continue;
            }

            // Obtener la mejor transformación CALCULOS DE MATRICES!!!!!!!!
            // Obtener los inliers
		
            std::vector<int> inliers;
            ransac.getInliers(inliers);

            //ESTO HAY QUE REVISARLO, peta fuerte

            // Crear nubes de puntos con los inliers
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud_current(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud_previous(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::copyPointCloud(*cloud_filtered, inliers, *inlier_cloud_current);
            pcl::copyPointCloud(*prev_cloud, inliers, *inlier_cloud_previous);

            // Calcular la transformación rígida entre los inliers
            Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
            Eigen::Vector4f centroid_current, centroid_previous;
            pcl::compute3DCentroid(*inlier_cloud_current, centroid_current);
            pcl::compute3DCentroid(*inlier_cloud_previous, centroid_previous);

            // Centrar los puntos en el origen
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_centered(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_previous_centered(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::demeanPointCloud(*inlier_cloud_current, centroid_current, *cloud_current_centered);
            pcl::demeanPointCloud(*inlier_cloud_previous, centroid_previous, *cloud_previous_centered);

            // Calcular la matriz de covarianza manualmente
            Eigen::Matrix3f covariance_matrix = computeCovarianceMatrixNormalized(*cloud_previous_centered, *cloud_current_centered);

            // Descomposición SVD para obtener la rotación
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3f rotation_matrix = svd.matrixV() * svd.matrixU().transpose();

            // Calcular la traslación
            Eigen::Vector3f translation_vector = centroid_previous.head<3>() - rotation_matrix * centroid_current.head<3>();

            // Construir la matriz de transformación
            transformation.block<3, 3>(0, 0) = rotation_matrix;
            transformation.block<3, 1>(0, 3) = translation_vector;

            // Acumular la transformación
            TT = transformation * TT;  // Acumular la transformación

            // Aplicar la transformación total TT a la nube de puntos transformada
            pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, TT);

            // Acumular los datos transformados en el mapa M
            *prev_cloud += *cloud_transformed;
        } else {
            // Si es la primera nube, solo guardamos los datos
            *prev_cloud = *cloud_filtered;
        }
	}
	// Al final, el mapa M tiene la nube de puntos alineada acumulada
    std::cout << "Tamaño de la nube de puntos alineada: " << prev_cloud->size() << std::endl;

	// Visualizar el mapa final
    pcl::visualization::PCLVisualizer viewer("Mapa Alineado");
    viewer.addPointCloud(prev_cloud, "cloud");  // Agregar la nube de puntos al visor
    viewer.setBackgroundColor(0, 0, 0);  // Fondo negro
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  // Establecer el tamaño de los puntos
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}
