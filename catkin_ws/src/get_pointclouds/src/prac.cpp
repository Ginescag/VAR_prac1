	
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
#include <pcl/features/fpfh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>


int main(int argc, char** argv) {
	ros::init(argc, argv, "sub_prac");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapa_global(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f TT = Eigen::Matrix4f::Identity();
	
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr prev_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr prev_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);

	for (int i = 0; i < 20; i++){
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

		// Detector de keypoints SIFT
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

		if (keypoints->empty()) {
			PCL_ERROR("Ningún keypoint extraído de la nube de puntos\n");
			return -1;
		} else {
			// Obtención de las normales a partir de los keypoints
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

			normal_estimator.setSearchMethod(tree);       // Método de búsqueda (KdTree)
			normal_estimator.setKSearch(30);              // Usar 30 vecinos más cercanos
			normal_estimator.setInputCloud(keypoints); // Nube filtrada (usar la original si es preciso)
			normal_estimator.compute(*normals); 

			//Aqui la lógica de los descriptores

			// Descriptor FPFH
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(new pcl::PointCloud<pcl::FPFHSignature33>);
			pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;

			// Configuración del descriptor FPFH
			fpfh.setInputCloud(keypoints);         // Keypoints detectados
			fpfh.setInputNormals(normals);         // Normales precalculadas de la nube
			fpfh.setSearchMethod(tree);
			fpfh.setRadiusSearch(0.05f);           // Radio de búsqueda (5 cm)
			fpfh.compute(*descriptors);
			
			if(i == 0){
				prev_descriptors = descriptors;
				prev_keypoints	 = keypoints;
			} else {
				// Emparejamiento de descriptores entre nubes consecutivas
				pcl::CorrespondencesPtr correspondencias(new pcl::Correspondences);
				pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
				est.setInputSource(descriptors);
				est.setInputTarget(prev_descriptors);
				est.determineCorrespondences(*correspondencias);

				// Eliminar emparejamientos erroneos con RANSAC

				// 1. Definir el objeto RANSAC para correspondencias
				pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> ransac_filter;

				// 2. Configurar parámetros clave
				ransac_filter.setInlierThreshold(0.02f);      // Máxima distancia para considerar inlier (2 cm)
				ransac_filter.setMaximumIterations(1000);     // Número de iteraciones de RANSAC
				ransac_filter.setRefineModel(true);           // Refinar el modelo con los inliers

				// 3. Proporcionar las coordenadas 3D de los keypoints
				ransac_filter.setInputSource(keypoints);   // Keypoints de la nube actual
				ransac_filter.setInputTarget(prev_keypoints); // Keypoints de la nube anterior

				// 4. Aplicar RANSAC a las correspondencias brutas
				pcl::CorrespondencesPtr correspondencias_filtradas(new pcl::Correspondences);
				ransac_filter.getRemainingCorrespondences(*correspondencias, *correspondencias_filtradas);

				// 5. Obtener la transformación estimada (opcional)
				Eigen::Matrix4f transformacion_actual = ransac_filter.getBestTransformation();

				TT = TT * transformacion_actual;

				
				// 1. Crear objeto de estimación SVD
				// pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> svd;

				// 2. Definir la matriz de transformación (inicialmente identidad)
				// Eigen::Matrix4f transformacion = Eigen::Matrix4f::Identity();

				// 3. Calcular la transformación
				// svd.estimateRigidTransformation(
				// 	*keypoints_actual,     // Keypoints de la nube actual (source)
				// 	*keypoints_anterior,   // Keypoints de la nube anterior (target)
				// 	*correspondencias_filtradas, // Correspondencias válidas (inliers)
				// 	transformacion         // Matriz de salida
				// );
			}

		}

	// 	// 4. Realizar la alineación de nubes de puntos utilizando RANSAC
    //     if (i > 0) {
    //         // Establecer el modelo de RANSAC (utilizamos un modelo de plano como ejemplo)
    //         pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(prev_cloud));
	// 		pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model);
    //         // Configurar el RANSAC
    //     // Método RANSAC
    //         ransac.setMaxIterations(1000);                // Número máximo de iteraciones
    //         ransac.setDistanceThreshold(0.01);            // Distancia máxima permitida entre puntos

    //         // Ejecutar RANSAC para encontrar la mejor transformación
    //         if (ransac.computeModel() == 0) {
    //             std::cerr << "RANSAC no pudo encontrar una buena transformación." << std::endl;
    //             continue;
    //         }

    //         // Obtener la mejor transformación CALCULOS DE MATRICES!!!!!!!!
    //         // Obtener los inliers
		
    //         std::vector<int> inliers;
    //         ransac.getInliers(inliers);

    //         //ESTO HAY QUE REVISARLO, peta fuerte

    //         // Crear nubes de puntos con los inliers
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud_current(new pcl::PointCloud<pcl::PointXYZRGB>);
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlier_cloud_previous(new pcl::PointCloud<pcl::PointXYZRGB>);

    //         pcl::copyPointCloud(*cloud_filtered, inliers, *inlier_cloud_current);
    //         pcl::copyPointCloud(*prev_cloud, inliers, *inlier_cloud_previous);

    //         // Calcular la transformación rígida entre los inliers
    //         Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    //         Eigen::Vector4f centroid_current, centroid_previous;
    //         pcl::compute3DCentroid(*inlier_cloud_current, centroid_current);
    //         pcl::compute3DCentroid(*inlier_cloud_previous, centroid_previous);

    //         // Centrar los puntos en el origen
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_current_centered(new pcl::PointCloud<pcl::PointXYZRGB>);
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_previous_centered(new pcl::PointCloud<pcl::PointXYZRGB>);

    //         pcl::demeanPointCloud(*inlier_cloud_current, centroid_current, *cloud_current_centered);
    //         pcl::demeanPointCloud(*inlier_cloud_previous, centroid_previous, *cloud_previous_centered);

    //         // Calcular la matriz de covarianza manualmente
    //         Eigen::Matrix3f covariance_matrix = computeCovarianceMatrixNormalized(*cloud_previous_centered, *cloud_current_centered);

    //         // Descomposición SVD para obtener la rotación
    //         Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //         Eigen::Matrix3f rotation_matrix = svd.matrixV() * svd.matrixU().transpose();

    //         // Calcular la traslación
    //         Eigen::Vector3f translation_vector = centroid_previous.head<3>() - rotation_matrix * centroid_current.head<3>();

    //         // Construir la matriz de transformación
    //         transformation.block<3, 3>(0, 0) = rotation_matrix;
    //         transformation.block<3, 1>(0, 3) = translation_vector;

    //         // Acumular la transformación
    //         TT = transformation * TT;  // Acumular la transformación

    //         // Aplicar la transformación total TT a la nube de puntos transformada
    //         pcl::transformPointCloud(*cloud_filtered, *cloud_transformed, TT);

    //         // Acumular los datos transformados en el mapa M
    //         *prev_cloud += *cloud_transformed;
    //     } else {
    //         // Si es la primera nube, solo guardamos los datos
    //         *prev_cloud = *cloud_filtered;
    //     }
	}
	// Al final, el mapa M tiene la nube de puntos alineada acumulada
    // std::cout << "Tamaño de la nube de puntos alineada: " << prev_cloud->size() << std::endl;

	// // Visualizar el mapa final
    // pcl::visualization::PCLVisualizer viewer("Mapa Alineado");
    // viewer.addPointCloud(prev_cloud, "cloud");  // Agregar la nube de puntos al visor
    // viewer.setBackgroundColor(0, 0, 0);  // Fondo negro
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");  // Establecer el tamaño de los puntos
    // while (!viewer.wasStopped()) {
    //     viewer.spinOnce();
    // }

    return 0;
}
