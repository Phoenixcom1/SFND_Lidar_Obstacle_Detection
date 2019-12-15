// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <cmath>
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>());


    for(int index : inliers->indices)
        cloud_f->points.push_back(cloud->points[index]);

    // Create the filtering object
        pcl::ExtractIndices<PointT> extract;

        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;

	// Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneSelf(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	//initialize the random seed
	std::srand(std::time(nullptr));

	/*
	 * Ensuring that the input cloud is big enough to find a model
	 * */
	if(cloud->points.size() > 3){

		while(maxIterations--){

			std::unordered_set<int> inliers_tmp;
			//inliers_tmp.clear();

			/*
			 * Picking randomly two points while making sure that a point didn't got choosen twice (items of set must be unique to be added)
			 * */
			while(inliers_tmp.size() < 3){
				inliers_tmp.insert(rand() % cloud->points.size());
			}
			// Randomly sample subset and fit line
			auto itr = inliers_tmp.begin();
			int idx_p1 = *itr;
			int idx_p2 = *(++itr);
			int idx_p3 = *(++itr);

			auto p1 = cloud->points[idx_p1];
			auto p2 = cloud->points[idx_p2];
			auto p3 = cloud->points[idx_p3];


			/*
			 *Defining vector 1
			 * v1 = < 	x2−x1,
			 * 			y2−y1,
			 * 			z2−z1	>
			 * */
			std::vector<double> v1{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
			/*
			 *Defining vector 2
			 * v2 = <	x3−x1,
			 * 			y3−y1,
			 * 			z3−z1	>
			 * */
			std::vector<double> v2{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

			/*
			 * Calculating the normal vector for the plane
			 * by cross product of	vn = v1 × v2 =
			 *			<
			 *				(v1.y * v2.z) - (v1.z * v2.y),
			 *				(v1.z * v2.x) - (v1.x * v2.z),
			 *				(v1.x * v2.y) - (v1.y * v2.x)	>
			 *		=
			 * 			<	(y2−y1)(z3−z1) − (z2−z1)(y3−y1),
			 * 				(z2-z1)(x3-x1) - (x2-x1)(z3-z1),
			 * 				(x2−x1)(y3−y1) − (y2−y1)(x3−x1)	>
			 *
			 * 				vn = v1 × v2 = < i,j,k >
			 * */

			/*
			 * plane formula
			 * Ax + By + Cz + D = 0
			 * for given values:
			 *
			 * 		i(x−x1) + j(y−y1) + k(z−z1) = 0,
			 * 		ix + jy + kz − (ix1 + jy1 + kz1) = 0
			 * 			A=i,
			 * 			B=j,
			 * 			C=k,
			 * 			D=−(ix1+jy1+kz1)
			 *
			 */
			double a = v1[1] * v2[2] - v1[2] * v2[1];
			double b = v1[2] * v2[0] - v1[0] * v2[2];
			double c = v1[0] * v2[1] - v1[1] * v2[0];
			double d = -(a * p1.x + b * p1.y + c * p1.z);

			//std::cout << "a " << a << std::endl;

			// Measure distance between every point and fitted line
			for (size_t i = 0; i < cloud->points.size(); ++i) {
				const auto &p = cloud->points[i];
				/*
				 * calculating distance for a point to the line model
				 * Distance d = |Ax+By+Cz+D|/sqrt(A^2+B^2+C^2)
				 * */
				double dis = std::abs(a * p.x + b * p.y + c * p.z + d) / std::sqrt(a * a + b * b + c * c);
				// If distance is smaller than threshold count this index as inlier
				if (dis < distanceThreshold) {
				  inliers_tmp.insert(i);
				}
			}
			/*
			 * If this round has more inliers and therefore a better model, take it as best fit
			 */
			if (inliersResult.size() < inliers_tmp.size()) {
				inliersResult = inliers_tmp;
			}
		}
	}
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
	inliers->indices = std::vector<int>(inliersResult.begin(), inliersResult.end());


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
