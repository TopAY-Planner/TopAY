#pragma once

#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

using namespace std;

namespace nmoma_planner {
namespace random_map {
    struct Box {
        typedef std::array<float, 8> array_repr;

        Eigen::Vector3d pos;
        Eigen::Vector3d size;
        double theta;
        std::array<Eigen::Vector3d, 8> corners;

        Box(const Eigen::Vector3d& pos, const Eigen::Vector3d& size, double theta = 0.0) : pos(pos), size(size), theta(theta) {
            Eigen::Matrix3d rotation_matrix;

            rotation_matrix << 
                cos(theta), -sin(theta), 0, 
                sin(theta),  cos(theta), 0, 
                0, 0, 1;
            
            corners[0] = pos;
            corners[1] = pos + rotation_matrix * Eigen::Vector3d(size.x(), 0, 0);
            corners[2] = pos + rotation_matrix * Eigen::Vector3d(0, size.y(), 0);
            corners[3] = pos + rotation_matrix * Eigen::Vector3d(size.x(), size.y(), 0);
            
            corners[4] = corners[0] + size.z() * Eigen::Vector3d (0, 0, 1.0);
            corners[5] = corners[1] + size.z() * Eigen::Vector3d (0, 0, 1.0);
            corners[6] = corners[2] + size.z() * Eigen::Vector3d (0, 0, 1.0);
            corners[7] = corners[3] + size.z() * Eigen::Vector3d (0, 0, 1.0);
        }

        pcl::PointCloud<pcl::PointXYZ> generatePCL(double resolution) const;

        inline array_repr toArray() const {
            return { pos.x(), pos.y(), pos.z(), size.x(), size.y(), size.z(), cos(theta), sin(theta) };
        }

        inline bool overlap2d(const Box& other) const {
            // Project all corners onto axes of both boxes
            std::array<Eigen::Vector2d, 4> axes = {
                (corners[1] - corners[0]).head<2>().normalized(),
                (corners[2] - corners[0]).head<2>().normalized(),
                (other.corners[1] - other.corners[0]).head<2>().normalized(),
                (other.corners[2] - other.corners[0]).head<2>().normalized()
            };
        
            for (const auto& axis : axes) {
                double min1 = INFINITY, max1 = -INFINITY;
                double min2 = INFINITY, max2 = -INFINITY;
                for (int i = 0; i < 4; i++) {
                    double proj1 = corners[i].head<2>().dot(axis);
                    double proj2 = other.corners[i].head<2>().dot(axis);
                    min1 = std::min(min1, proj1);
                    max1 = std::max(max1, proj1);
                    min2 = std::min(min2, proj2);
                    max2 = std::max(max2, proj2);
                }
                if (max1 < min2 || max2 < min1) return false;
            }
            return true;
        }

        inline bool overlap(const Box& other) const {
            return overlap2d(other) && 
                (pos.z() + size.z() > other.pos.z() && 
                pos.z() < other.pos.z() + other.size.z());
        }
    };

    struct RandomPCGenerator {
        // random
        std::random_device rd;
        std::mt19937 eng;
        uniform_real_distribution<double> rand_x;
        uniform_real_distribution<double> rand_y;
        uniform_real_distribution<double> rand_wall_size;
        uniform_real_distribution<double> rand_wall_height;
        uniform_real_distribution<double> rand_float_size;
        uniform_real_distribution<double> rand_float_height;
        uniform_real_distribution<double> rand_theta;

        uniform_real_distribution<double> rand_desk_width;
        uniform_real_distribution<double> rand_desk_length;
        uniform_real_distribution<double> rand_desk_height;

        uniform_int_distribution<int> rand_arragement;

        // params
        vector<int> obs_num = {1, 1};
        vector<double> wall_size_range = {0.1, 0.1};
        vector<double> wall_height_range = {0.1, 0.1};
        vector<double> float_size_range = {0.1, 0.1};
        vector<double> float_height_range = {0.1, 0.1};
        // vector<double> theta_range = {-M_PI, M_PI};
        double resolution = 0.1;
        double size_x = 30.0;
        double size_y = 30.0;
        double min_obs_dis = 1.0;

        vector<double> desk_width_range = {0.75, 1.25};
        vector<double> desk_length_range= {0.75, 1.5};
        vector<double> desk_height_range={0.75, 1.5};
        vector<int> desk_arrangement_range = {1, 2};

        void init(
            const vector<int>& obs_num,
            const vector<double>& wall_size_range,
            const vector<double>& wall_height_range,
            const vector<double>& float_size_range,
            const vector<double>& float_height_range,
            double resolution,
            double size_x,
            double size_y,
            double min_obs_dis
        );
        void init(ros::NodeHandle& nh);
        
        pcl::PointCloud<pcl::PointXYZ> generateBox(const Eigen::Vector3d& size);

        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateDesk(
            const Eigen::Vector3d& pos,
            const Eigen::Vector3d& size,
            double theta);
        
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateDesk(
            const Eigen::Vector3d& pos,
            const Eigen::Vector3d& size,
            double theta, 
            const Eigen::Vector2i& arrangement);
            
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateDeskCase();
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateDeskCase(std::vector<Box> obs);
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generataRandomCaseAux();
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateRandomCase();
        std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<Box::array_repr>> generateRandomCase(unsigned int seed);
    };
}
}