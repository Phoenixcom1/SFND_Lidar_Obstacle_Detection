// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
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

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::unordered_set<int> inliers_tmp;
    	//initialize the random seed
    	srand ( time(NULL) );
    	// model points and parameters
    	int idx_p1, idx_p2, idx_p3;
    	float a,b,c,d,dis;
    	std::vector<float> v1,v2,vn;

    	/*
    	 * Ensuring that the input cloud is big enough to find a model
    	 * */
    	if(cloud->points.size() > 3){

    		while(maxIterations--){
    			inliers_tmp.clear();

    			/*
    			 * Picking randomly two points while making sure that a point didn't got choosen twice (items of set must be unique to be added)
    			 * */
    			while(inliers_tmp.size() < 3){
    				inliers_tmp.insert(rand() % cloud->points.size());
    			}
    			// Randomly sample subset and fit line
    			auto itr = inliers_tmp.begin();
    			idx_p1 = *itr;
    			*itr++;
    			idx_p2 = *itr;
    			*itr++;
    			idx_p3 = *itr;


    			/*
    			 * Defining vector 1
    			 * v1 = < 	x2−x1,
    			 * 			y2−y1,
    			 * 			z2−z1	>
    			 * */
    			v1.push_back(cloud->points[idx_p2].x - cloud->points[idx_p1].x);
    			v1.push_back(cloud->points[idx_p2].y - cloud->points[idx_p1].y);
    			v1.push_back(cloud->points[idx_p2].z - cloud->points[idx_p1].z);

    			/*
    			 * Defining vector 2
    			 * v2 = <	x3−x1,
    			 * 			y3−y1,
    			 * 			z3−z1	>
    			 * */
    			v2.push_back(cloud->points[idx_p3].x - cloud->points[idx_p1].x);
    			v2.push_back(cloud->points[idx_p3].y - cloud->points[idx_p1].y);
    			v2.push_back(cloud->points[idx_p3].z - cloud->points[idx_p1].z);

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

    			vn.push_back(v1[1] * v2[2] - v1[2] * v2[1]);
    			vn.push_back(v1[2] * v2[0] - v1[0] * v2[2]);
    			vn.push_back(v1[0] * v2[1] - v1[1] * v2[0]);

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
    			a = vn[0];
    			b = vn[1];
    			c = vn[2];
    			d = -(vn[0]*cloud->points[idx_p1].x + vn[1]*cloud->points[idx_p1].y + vn[2]*cloud->points[idx_p1].z);

    			// Measure distance between every point and fitted line
    			for(int i = 0; i < cloud->points.size(); i++){
    				/*
    				 * calculating distance for a point to the line model
    				 * Distance d = |Ax+By+Cz+D|/sqrt(A^2+B^2+C^2)
    				 * */
    				dis = std::fabs(a*cloud->points[i].x + b*cloud->points[i].y + c*cloud->points[i].z + d) / std::sqrt(std::pow(a,2) + std::pow(b,2) + std::pow(c,2));

    				// If distance is smaller than threshold count this index as inlier
    				if(dis <= distanceThreshold){
    					inliers_tmp.insert(i);
    				}
    			}
    			/*
    			 * If this round has more inliers and therefore a better model, take it as best fit
    			 */

    			if(inliers->indices.size() < inliers_tmp.size()){
    				inliers->indices.clear();
    				inliers->indices.insert(inliers->indices.end(), inliers_tmp.begin(), inliers_tmp.end());
    			}

    		}

    	}



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
