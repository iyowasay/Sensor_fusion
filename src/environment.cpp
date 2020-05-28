/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <memory>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)//, ProcessPointClouds<pcl::PointXYZI>* point_processor2, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
{
    // RENDER OPTIONS
    bool render_real_data = false;
    bool renderFiltered = false;
    bool renderSegment = true;
    bool render_obstacle = true;// bool render_road = true;
    bool render_cluster = false;
    bool render_box =false;

    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> point_processor2(new ProcessPointClouds<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = point_processor2->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    if(render_real_data)
        renderPointCloud(viewer, input_cloud, "inputCloud");

    float vox_resolution = 0.3f; // changing the size of voxel to change resolution(one point cloud point per voxel) 
    // define the region you can see
    std::vector<float> minsize{-20, -6, -2.5};
    std::vector<float> maxsize{30, 7, 2.5};
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processor2->FilterCloud(input_cloud, vox_resolution, Eigen::Vector4f(minsize[0],minsize[1],minsize[2],1.0), Eigen::Vector4f(maxsize[0],maxsize[1],maxsize[2],1.0));
    if(renderFiltered)
        renderPointCloud(viewer, filtered_cloud, "filtered_cloud");


    // segmentation
    float threshold = 0.25;
    int max_iter = 50;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_result = point_processor2->SegmentPlane(filtered_cloud, max_iter, threshold);

    if(renderSegment)
    {
        if(render_obstacle)
            renderPointCloud(viewer, segment_result.first, "obstacle_cloud", Color(1,0,0));
        renderPointCloud(viewer, segment_result.second, "road_cloud", Color(0,1,0));
    }
    // clustering
    int maxSize = 500;
    int minSize = 10;
    float dis_limit = 0.5;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = point_processor2->Clustering(segment_result.first, dis_limit, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,1), Color(1,1,0), Color(0,1,1), Color(192/255,192/255,192/255)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if(render_cluster)
        {
            std::cout <<"cluster size ";
            point_processor2->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%(colors.size())]);
        }
        
        if(render_box)
        {
            Box box = point_processor2->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        
        ++clusterId;
    }
    
    
  

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obstacle = true;
    bool render_road = true;
    bool render_cluster = false;
    bool render_box = false;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    // TODO:: Create lidar sensor
    double init_slope= 0;
    std::unique_ptr<Lidar> lidar1(new Lidar(cars, init_slope));
    pcl::PointCloud<pcl::PointXYZ>::Ptr ray_casting = lidar1->scan();
    //renderRays(viewer, lidar1->position, ray_casting);
    Color color(0,1,0);
    //renderPointCloud(viewer, ray_casting, "ray1", color);
    
  
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* point_processor1;
    float threshold = 0.2;
    int max_iter = 60;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_result = point_processor1->SegmentPlane(ray_casting, max_iter, threshold);

    if(render_obstacle)
        renderPointCloud(viewer, segment_result.first, "obstacle_cloud", Color(1,0,0));
    if(render_road)
        renderPointCloud(viewer, segment_result.second, "road_cloud", Color(0,1,0));

    
    int maxSize = 100;
    int minSize = 3;
    float dis_limit = 1.5;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = point_processor1->Clustering(segment_result.first, dis_limit, minSize, maxSize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,1,1), Color(192/255,192/255,192/255)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_cluster)
        {
            std::cout <<"cluster size ";
            point_processor1->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%(colors.size())]);
        }
        
        if(render_box)
        {
            Box box = point_processor1->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        
        ++clusterId;
    }
    
    
  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    ProcessPointClouds<pcl::PointXYZI>* point_processor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud1;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce ();
    }

    // while (!viewer->wasStopped ())
    // {
    //     viewer->removeAllPointClouds();
    //     viewer->removeAllShapes();
    //     input_cloud1 = point_processor->loadPcd((*streamIterator).string());
    //     cityBlock(viewer, point_processor, input_cloud1);

    //     streamIterator++;
    //     if(streamIterator == stream.end())
    //         streamIterator = stream.begin();
    //     viewer->spinOnce ();
    // }
}
