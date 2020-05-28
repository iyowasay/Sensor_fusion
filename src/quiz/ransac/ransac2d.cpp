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

std::unordered_set<int> Ransac_2d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    while(maxIterations--)
    {
        std::unordered_set<int> cur_inliers;
        while(cur_inliers.size()<2){
            cur_inliers.insert(rand() % cloud->points.size());
        }
        auto itr = cur_inliers.begin();
        pcl::PointXYZ p1 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p2 = cloud->points[*itr];
        
        float A = p1.y-p2.y;
        float B = p2.x-p1.x;
        float C = p1.x*p2.y-p1.y*p2.x;
    
        int idx = 0;
        for(auto p : cloud->points){
            if(cur_inliers.count(idx)>0){
                idx++;
                continue;
            }
            
            float distance = fabs(p.x*A+p.y*B+C)/sqrt(A*A+B*B);
            if(distance <= distanceTol){
                cur_inliers.insert(idx);
            }
            idx++;
        }
        
        inliersResult = inliersResult.size() < cur_inliers.size()? cur_inliers:inliersResult;

    }
    
    return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    while(maxIterations--)
    {
        std::unordered_set<int> cur_inliers;
        while(cur_inliers.size()<3){
            cur_inliers.insert(rand() % cloud->points.size());
        }
        auto itr = cur_inliers.begin();
        pcl::PointXYZ p1 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p2 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ p3 = cloud->points[*itr];
        
        float A = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
        float B = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
        float C = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
        float D = -(p1.x*A+p1.y*B+p1.z*C);
    
        int idx = 0;
        for(auto p : cloud->points){
            if(cur_inliers.count(idx)>0){
                idx++;
                continue;
            }
            
            float distance = fabs(p.x*A+p.y*B+p.z*C)/sqrt(A*A+B*B+C*C);
            if(distance <= distanceTol){
                cur_inliers.insert(idx);
            }
            idx++;
        }
        
        inliersResult = inliersResult.size() < cur_inliers.size()? cur_inliers:inliersResult;

    }
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "my RANSAC: " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;

}

int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
    

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2);

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

