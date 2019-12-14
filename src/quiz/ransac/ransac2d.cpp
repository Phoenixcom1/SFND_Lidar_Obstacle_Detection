/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <iostream>
#include <stdlib.h>
#include <time.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

	std::unordered_set<int> inliersResult, inliers_tmp;
	//initialize the random seed
	srand ( time(NULL) );
	// model points and parameters
	int idx_p1, idx_p2;
	float a,b,c,dis;

	/*
	 * Ensuring that the input cloud is big enough to find a model
	 * */
	if(cloud->points.size() > 2){

		while(maxIterations--){
			inliers_tmp.clear();

			/*
			 * Picking randomly two points while making sure that a point didn't got choosen twice (items of set must be unique to be added)
			 * */
			while(inliers_tmp.size() < 2){
				inliers_tmp.insert(rand() % cloud->points.size());
			}
			// Randomly sample subset and fit line
			auto itr = inliers_tmp.begin();
			idx_p1 = *itr;
			*itr++;
			idx_p2 = *itr;

			/*
			 * Line formula
			 * Ax + By + C = 0
			 * for two given points:
			 * (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0
			 */
			a = cloud->points[idx_p1].y - cloud->points[idx_p2].y;
			b = cloud->points[idx_p2].x - cloud->points[idx_p1].x;
			c = cloud->points[idx_p1].x * cloud->points[idx_p2].y - cloud->points[idx_p2].x * cloud->points[idx_p1].y;

			// Measure distance between every point and fitted line
			for(int i = 0; i < cloud->points.size(); i++){
				/*
				 * calculating distance for a point to the line model
				 * Distance d = |Ax+By+C|/sqrt(A^2+B^2)
				 * */
				dis = std::fabs(a*cloud->points[i].x + b*cloud->points[i].y + c) / std::sqrt(std::pow(a,2) + std::pow(b,2));

				// If distance is smaller than threshold count this index as inlier
				if(dis <= distanceTol){
					inliers_tmp.insert(i);
				}
			}
			/*
			 * If this round has more inliers and therefore a better model, take it as best fit
			 */
			if(inliersResult.size() < inliers_tmp.size()){
				inliersResult = inliers_tmp;
			}

		}

	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
