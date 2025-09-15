#include "random_map_generator/random_map.hpp"

#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using nmoma_planner::random_map::RandomPCGenerator;
using nmoma_planner::random_map::Box;

// pcl
std::vector<Eigen::Vector2d> centers;
pcl::PointCloud<pcl::PointXYZ> cloud_map;
std::vector<Box::array_repr> obs_boxes;

// ros
ros::Timer vis_timer;
ros::Publisher global_map_pub;
ros::Publisher global_box_pub;
sensor_msgs::PointCloud2 global_msg;
std_msgs::Float32MultiArray global_box_msg;

// params
bool has_map = false;
bool fix_generator = false;
double resolution = 0.1;
double size_x = 30.0;
double size_y = 30.0;
double vis_rate = 10.0;

pcl::PointCloud<pcl::PointXYZ> generateBox(const Eigen::Vector3d& size)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_box;
    int x_num, y_num, z_num;
    x_num = ceil(size.x()/resolution);
    y_num = ceil(size.y()/resolution);
    z_num = ceil(size.z()/resolution);
    pcl::PointXYZ pt;
    for (int i=0; i<x_num; i++)
        for (int j=0; j<y_num; j++)
            for (int k=0; k<z_num; k++)
            {
                pt.x = i * resolution;
                pt.y = j * resolution;
                pt.z = k * resolution;
                if (i==0 || j==0 || k==0 || i==x_num-1 || j==y_num-1 || k==z_num-1)
                    cloud_box.points.push_back(pt);
            }
    return cloud_box;
}

void generateRealCase()
{
    pcl::PointXYZ pt_random;

    pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(Eigen::Vector3d(size_x, resolution*2.0, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x - size_x / 2.0;
        pt_random.y = cloud_box.points[i].y + size_y / 2.0;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.y = cloud_box.points[i].y - size_y / 2.0;
        cloud_map.points.push_back(pt_random);
    }
    cloud_box = generateBox(Eigen::Vector3d(resolution*2.0, size_y, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x + size_x / 2.0;
        pt_random.y = cloud_box.points[i].y - size_y / 2.0;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.x = cloud_box.points[i].x - size_x / 2.0;
        cloud_map.points.push_back(pt_random);
    }

    Eigen::Vector3d box_size1(0.6, 0.6, 0.6);
    Eigen::Vector3d box_size2(0.6, 2.2, 0.6);
    Eigen::Vector3d box_size3(0.6, 0.6, 2.0);
    std::vector<Eigen::Vector3d> box_sizes {box_size1,
                                            box_size1,
                                            box_size2,
                                            box_size3,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1};
    std::vector<double> xs {-0.3, -0.3, -0.3, -2.0, -2.0, 2.0, 2.0, 0.0, 1.0, -3.1, -3.5};
    std::vector<double> ys {-0.8, 0.8, -0.8, 2.0, -2.0, 1.0, -2.0, -2.5, -2.0, 1.2, 0.3};
    std::vector<double> hs {0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int j=0; j<xs.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(box_sizes[j]);
        for (size_t i=0; i<cloud_box.points.size(); i++)
        {
            pt_random.x = cloud_box.points[i].x + xs[j];
            pt_random.y = cloud_box.points[i].y + ys[j];
            pt_random.z = cloud_box.points[i].z + hs[j];
            cloud_map.points.push_back(pt_random);
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

    return;
}

void generatePickPlaceCase()
{
    pcl::PointXYZ pt_random;

    pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(Eigen::Vector3d(size_x, resolution*2.0, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x - size_x / 2.0;
        pt_random.y = cloud_box.points[i].y + size_y / 2.0;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.y = cloud_box.points[i].y - size_y / 2.0;
        cloud_map.points.push_back(pt_random);
    }
    cloud_box = generateBox(Eigen::Vector3d(resolution*2.0, size_y, 1.0));
    for (size_t i=0; i<cloud_box.points.size(); i++)
    {
        pt_random.x = cloud_box.points[i].x + size_x / 2.0;
        pt_random.y = cloud_box.points[i].y - size_y / 2.0;
        pt_random.z = cloud_box.points[i].z;
        cloud_map.points.push_back(pt_random);
        pt_random.x = cloud_box.points[i].x - size_x / 2.0;
        cloud_map.points.push_back(pt_random);
    }

    Eigen::Vector3d box_size1(0.6, 0.6, 0.6);
    Eigen::Vector3d box_size2(0.6, 2.4, 0.6);
    Eigen::Vector3d box_size3(0.6, 0.6, 2.0);
    std::vector<Eigen::Vector3d> box_sizes {box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size1,
                                            box_size2};
    std::vector<double> xs {4.0, 12.6, 4.0, 4.0, 6.5 , 6.5, 6.5, 6.5};
    std::vector<double> ys {-0.55, -0.55, 0.05, -2.2, -2.2, -0.8, 1.8, -0.4};
    std::vector<double> hs {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6};

    for (int j=0; j<xs.size(); j++)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_box = generateBox(box_sizes[j]);
        for (size_t i=0; i<cloud_box.points.size(); i++)
        {
            pt_random.x = cloud_box.points[i].x + xs[j];
            pt_random.y = cloud_box.points[i].y + ys[j];
            pt_random.z = cloud_box.points[i].z + hs[j];
            cloud_map.points.push_back(pt_random);
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

    std::string file_name = ros::package::getPath("random_map_generator")+"/env/map.pcd";
    pcl::io::savePCDFile(file_name.c_str(), cloud_map);

    return;
}

void generateFileCase()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("random_map_generator")+"/env/map.pcd", *cloudFiltered) == -1)
    {
        PCL_ERROR("Failed to read PCD file.\n");
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloudFiltered);
    vg.setLeafSize(resolution, resolution, resolution); // 设置体素网格大小
    vg.filter(cloud_map);

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";
}

void visCallback(const ros::TimerEvent &e)
{
    if (!has_map)
        return;
    global_map_pub.publish(global_msg);
    global_box_pub.publish(global_box_msg);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "random_map_node");
    ros::NodeHandle nh("~");

    int case_id = 0;
	nh.getParam("/map/case_id", case_id);
	nh.getParam("/map/fix_generator", fix_generator);
	nh.getParam("/map/size_x", size_x);
	nh.getParam("/map/size_y", size_y);
	nh.getParam("/map/resolution", resolution);
	nh.getParam("/map/vis_rate", vis_rate);

    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    global_box_pub = nh.advertise<std_msgs::Float32MultiArray>("global_boxes", 1);

    RandomPCGenerator generator;
    std::cout << "Generating case " << case_id << "..." << std::endl;
    switch (case_id)
    {
        case 1:
            generateRealCase();
            break;
        case 2:
            generateFileCase();
            break;
        case 3:
            generatePickPlaceCase();
            break;
        case 4:
        {
            generator.init(nh);
            std::tie(cloud_map, obs_boxes) = generator.generateDeskCase();

            // Publish and save point cloud
            cloud_map.width = cloud_map.points.size();
            cloud_map.height = 1;
            cloud_map.is_dense = true;
            pcl::toROSMsg(cloud_map, global_msg);
            global_msg.header.frame_id = "world";
            
            
            std::string file_name = ros::package::getPath("random_map_generator")+"/env/map.pcd";
            pcl::io::savePCDFile(file_name.c_str(), cloud_map);
            
            // Publish boxes representation
            global_box_msg.data.clear();
            for(auto &box : obs_boxes)
            for(size_t i = 0; i < 8; i++){
                global_box_msg.data.push_back(box[i]);
            }
            global_box_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            global_box_msg.layout.dim[0].label = "height";
            global_box_msg.layout.dim[0].size = obs_boxes.size();
            global_box_msg.layout.dim[0].stride = 8;
            has_map = true;
            break;
        }

        default:
        {
            std::cout << "Generating random case..." << std::endl;
            generator.init(nh);
            if (!fix_generator)
                std::tie(cloud_map, obs_boxes) = generator.generateRandomCase();
            else
                std::tie(cloud_map, obs_boxes) = generator.generateRandomCase(42);

            // Publish and save point cloud
            cloud_map.width = cloud_map.points.size();
            cloud_map.height = 1;
            cloud_map.is_dense = true;
            has_map = true;

            pcl::toROSMsg(cloud_map, global_msg);
            global_msg.header.frame_id = "world";

            std::string file_name = ros::package::getPath("random_map_generator")+"/env/map.pcd";
            pcl::io::savePCDFile(file_name.c_str(), cloud_map);

            // Publish boxes representation
            global_box_msg.data.clear();
            for(auto &box : obs_boxes)
            for(size_t i = 0; i < 8; i++){
                global_box_msg.data.push_back(box[i]);
            }
            global_box_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            global_box_msg.layout.dim[0].label = "height";
            global_box_msg.layout.dim[0].size = obs_boxes.size();
            global_box_msg.layout.dim[0].stride = 8;
            break;
        }
    }

    vis_timer = nh.createTimer(ros::Duration(1.0/vis_rate), visCallback);

    ros::spin();

    return 0;
}