/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

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
    std::unordered_set<int> inliersResult;
  	srand(time(NULL));
  	for(int iter = 0; iter < maxIterations; iter++)
    {
      //Randomly sample subset and fit line
      //A set will contain unique elements, using a set avoids picking the same point index twice
      std::unordered_set<int> inliersResultTemporal;
      while (inliersResultTemporal.size() < 3)
      {
        int random = rand() % cloud->points.size();
        inliersResultTemporal.insert(random);
      }
      auto itr = inliersResultTemporal.begin();
      pcl::PointXYZ p1 = cloud->points[*itr];
      itr++;
      pcl::PointXYZ p2 = cloud->points[*itr];
      itr++;
      pcl::PointXYZ p3 = cloud->points[*itr];
      
	  // Measure distance between every point and fitted line
	  for(int index = 0; index < cloud->points.size(); index++)
	  {
        auto point = cloud->points[index];
	    float A = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
	    float B = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
	    float C = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
        float D = -(A*p1.x+B*p1.y+C*p1.z);
        float distance = abs(A*point.x+B*point.y+C*point.z+D)/(sqrt(pow(A,2.0)+pow(B,2.0)+pow(C,2.0)));
        // If distance is smaller than threshold count it as inlier

        if(distance<distanceTol)
	    {
		  inliersResultTemporal.insert(index);
	    }
	  }
	  if(inliersResultTemporal.size()>inliersResult.size())
	  {
		inliersResult = inliersResultTemporal;
	  }
    }
  	return inliersResult;
  
  	// ------------------ Ransac on 2D ------------------------------
	/*std::unordered_set<int> inliersResult;
	srand(time(NULL));
	// TODO: Fill in this function
	// For max iterations 
	for(int iter = 0; iter<maxIterations; iter++)
    {
	// Randomly sample subset and fit line
      //The set is going to contain unique elements, avoids picking the same point index twice
	  std::unordered_set<int> inliersResultTemporal;
	  while (inliersResultTemporal.size() < 2)
      {
        int random = rand() % cloud->points.size();
        inliersResultTemporal.insert(random);        
      }
      auto itr = inliersResultTemporal.begin();
      pcl::PointXYZ p0 = cloud->points[*itr];
      itr++;
      pcl::PointXYZ p1 = cloud->points[*itr];
      
	  // Measure distance between every point and fitted line
	  for(int index = 0; index < cloud->points.size(); index++)
	  {
        auto point = cloud->points[index];
	    float distance = abs((p0.y-p1.y)*point.x + (p1.x-p0.x)*point.y + p0.x*p1.y - p1.x*p0.y)/sqrt(pow(p0.y-p1.y, 2)+pow(p1.x-p0.x, 2));
        // If distance is smaller than threshold count it as inlier
	    
        if(distance<distanceTol)
	    {
		  inliersResultTemporal.insert(index);
	    }
	  }
	  if(inliersResultTemporal.size()>inliersResult.size())
	  {
		inliersResult = inliersResultTemporal;
	  }
	}
	// Return indicies of inliers from fitted line with most inliers
  	std::cout << "Number of inliers: " << inliersResult.size() << std::endl;
	return inliersResult;*/

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 20, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
