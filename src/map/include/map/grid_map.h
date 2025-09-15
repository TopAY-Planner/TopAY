#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <random>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fake_moma/moma_param.h>
// #include "utils/random_pc_gen.hpp"
#include "random_map_generator/random_map.hpp"
#include "rog_map/rog_map.h"

#define IN_CLOSE 97
#define IN_OPEN 98
#define IS_UNKNOWN 99

using namespace std;

using nmoma_planner::random_map::Box;
using nmoma_planner::random_map::RandomPCGenerator;

#define GET_PARAM_OR_THROW(nh, param_name, param_var) \
    do { \
        if (!(nh).getParam((param_name), (param_var))) { \
            std::string error_msg = "Failed to get required parameter: " + std::string(param_name); \
            throw std::runtime_error(error_msg); \
        } \
    } while (0)

namespace nmoma_planner
{
    struct GridNode;
    typedef GridNode* GridNodePtr;

    struct GridNode
    {   
        Eigen::Vector2i index;
        double fScore;
        double gScore;
        GridNodePtr parent;
        char id;

        GridNode() : 
            index(Eigen::Vector2i::Zero()), 
            fScore(0.0), 
            gScore(0.0), 
            parent(nullptr), 
            id(IS_UNKNOWN) {}
        
        GridNode(const Eigen::Vector2i& _index) : 
            index(_index), 
            fScore(0.0), 
            gScore(0.0), 
            parent(nullptr), 
            id(IS_UNKNOWN) {}

        void reset()
        {
            parent = nullptr;
            gScore = fScore = 0.0;
            id = IS_UNKNOWN;
            return;
        }
        
        ~GridNode(){};
    };

    class GridMap
    {
        public:
            // params
            int             buffer_size_2d, buffer_size_3d;
            double          resolution, resolution_inv;
            Eigen::Vector3d map_origin;
            Eigen::Vector3d map_size;
            Eigen::Vector3d min_boundary;
            Eigen::Vector3d max_boundary;
            Eigen::Vector3i min_idx;
            Eigen::Vector3i max_idx;
            Eigen::Vector3i voxel_num;
            bool use_rog;
            rog_map::ROGMap::Ptr rog_map;

        private:
            //datas
            GridNodePtr* grid_node_map = nullptr;
            vector<char>   occ_buffer_3d;
            vector<char>   occ_buffer_2d;
            vector<char>   occ_buffer_2d_critical;
            vector<double> esdf_buffer_3d;
            vector<double> esdf_buffer_2d;
            vector<double> esdf_buffer_2d_inflate;
            vector<double> esdf_buffer_2d_critical;
            sensor_msgs::PointCloud2 esdf_cloud_3d;
            sensor_msgs::PointCloud2 esdf_cloud_2d;
            MomaParam moma_param;
            RandomPCGenerator map_gener;
            std::vector<Box::array_repr> boxes;

            pcl::PointCloud<pcl::PointXYZ> cloud_map;
            sensor_msgs::PointCloud2 cloud_msg;

            //ros
            bool       map_ready = false;
            ros::Timer vis_timer;
            ros::Publisher esdf_2d_pub;
            ros::Publisher esdf_3d_pub;
            ros::Subscriber cloud_sub;
            ros::Subscriber box_sub; // for box representation
            ros::Publisher global_map_pub;

            //statistical
            std::string stat_number;
            std::string stat_file;

            std::string mode;

            bool fixed_sequence;
            unsigned int map_seed = 42;
        public:
            GridMap() {}
            ~GridMap()
            { 
                for (int i=0; i<buffer_size_3d; i++) 
                    delete[] grid_node_map[i];

                delete[] grid_node_map;
                return;
            }

            void init(ros::NodeHandle& nh);
            void updateESDF();
            void visCallback(const ros::TimerEvent& /*event*/);
            void cloudCallback(const sensor_msgs::PointCloud2 msg);
            void boxCallback(const std_msgs::Float32MultiArray msg);
            void toMsg(void);
            // void toFile(void);
            void regenerateMap(void);
            void regenerateDesk(std::vector<Eigen::Vector2d> spawn = {});
            void loadMap(const vector<char>& occ_2d, const vector<char>& occ_3d);
            sensor_msgs::PointCloud2 getSurround(void);

            // 2D functions
            std::pair<std::vector<Eigen::Vector2d>, double> astarPlan2d(const Eigen::Vector2d& start, 
                                                                     const Eigen::Vector2d& end,
                                                                     const double& threshold = 0.0);
            inline void getDistance2d(const Eigen::Vector2d& pos, double& distance);
            inline void getDisWithGradI2d(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad, 
                                         bool inflate = false, bool critical = false);
            inline bool isCollision2d(const Eigen::Vector2d& pos, double threshold = 0.0);
            inline bool isCollisionIndx2d(int x, int y, double threshold = 0.0);
            inline bool isCollisionGrid2d(const Eigen::Vector2d& pos, double threshold = 0.0);
            inline bool isLineCollisionGrid2d(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double threshold = 0.0);
            inline bool isWholeBodyCollision(const Eigen::VectorXd& state);
            inline bool isWholeBodyCollision(const Eigen::VectorXd& state, int& coll_type);
            inline void boundIndex2d(Eigen::Vector2i& id);
            inline void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
            inline void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
            inline int toAddress2d(const Eigen::Vector2i& id);
            inline int toAddress2d(const int& x, const int& y);
            inline bool isInMap2d(const Eigen::Vector2d& pos);
            inline bool isInMap2d(const Eigen::Vector2i& idx);

            // 3D functions
            inline double getDistance(const Eigen::Vector3d& pos);
            inline double getDistance(const Eigen::Vector3i& id);
            inline std::pair<double, Eigen::Vector3d> evaluateEDTWithGrad(
                const Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad);
                
            inline void getVoxelNum(Eigen::Vector3i &voxel_num) { voxel_num = this->voxel_num; }
            inline Eigen::Vector2i pos2dToIndex(const Eigen::Vector2d &pos);
            inline void resetAstarBuffer() {return;}
            inline bool isInMap(const Eigen::Vector3d& pos) { return isInMap3d(pos); }
            inline bool isInMap(const Eigen::Vector3i& idx) { return isInMap3d(idx); }
            inline bool isInMap(const Eigen::Vector2d& pos) { return isInMap2d(pos); }
            inline bool isInMap(const Eigen::Vector2i& idx) { return isInMap2d(idx); }

            inline void getDistance3d(const Eigen::Vector3d& pos, double& distance);
            inline void getDisWithGradI3d(const Eigen::Vector3d& pos, double& distance, Eigen::Vector3d& grad);
            inline bool isCollision3d(const Eigen::Vector3d& pos, double threshold = 0.0);
            inline void boundIndex3d(Eigen::Vector3i& id);
            inline void posToIndex3d(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
            inline void indexToPos3d(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
            inline int toAddress3d(const Eigen::Vector3i& id);
            inline int toAddress3d(const int& x, const int& y, const int& z);
            inline bool isInMap3d(const Eigen::Vector3d& pos);
            inline bool isInMap3d(const Eigen::Vector3i& idx);
            inline double getDistCoarse2d(const Eigen::Vector2d& pos, bool critical = false);
            inline double getDistCoarse2i(const Eigen::Vector2i& id, bool critical = false);
            
            inline bool mapReady();
            inline int getXnum() { return voxel_num(0); }
            inline int getYnum() { return voxel_num(1); }
            inline double getResolution() { if (use_rog) return rog_map->getResolution(); return resolution; }
            inline Eigen::Vector2d getMinBound() { return min_boundary.head(2); }
            inline Eigen::Vector2d getMaxBound() { return max_boundary.head(2); }
            inline Eigen::Vector3d getOrigin() { if (use_rog) return rog_map->getLocalMapOrigin(); return map_origin; }

            inline Eigen::Vector3d getOffset() { if (use_rog) return Eigen::Vector3d::Zero(); return Eigen::Vector3d(0.5, 0.5, 0.5) - this->getOrigin() / resolution;}

            typedef shared_ptr<GridMap> Ptr;
            typedef unique_ptr<GridMap> UniPtr;

            // getter functions
            inline const vector<char>& getOccBuffer2d() const { return occ_buffer_2d; }
            inline const vector<char>& getOccBuffer3d() const { return occ_buffer_3d; }
            inline const vector<double>& getESDFBuffer2d() const { return esdf_buffer_2d; }
            inline const vector<double>& getESDFBuffer3d() const { return esdf_buffer_3d; }
            inline const vector<std::array<float, 8>>& getBoxes() const { return boxes; }
    };

    inline double GridMap::getDistance(const Eigen::Vector3d& pos)
    {
        Eigen::Vector3i id1;
        posToIndex3d(pos, id1);
        boundIndex3d(id1);
        return getDistance(id1);
    }

    inline double GridMap::getDistance(const Eigen::Vector3i& id)
    {
        Eigen::Vector3i id1 = id;
        boundIndex3d(id1);
        Eigen::Vector3d pos;
        indexToPos3d(id1, pos);
        double distance;
        getDistance3d(pos, distance);
        return distance;
    }

    inline std::pair<double, Eigen::Vector3d> GridMap::evaluateEDTWithGrad(
        const Eigen::Vector3d& pos, double& dist, Eigen::Vector3d& grad) {

        getDisWithGradI3d(pos, dist, grad);

        double _distance = dist;
        Eigen::Vector3d _grad = grad;
        return std::make_pair(_distance, _grad);
    }

    inline Eigen::Vector2i GridMap::pos2dToIndex(const Eigen::Vector2d &pos) {
        Eigen::Vector2i id;
        posToIndex2d(pos, id);
        return id;
    }

    inline void GridMap::getDistance2d(const Eigen::Vector2d& pos, double& distance)
    {
        if (use_rog) {
            // test if the point is inside the local map
            Eigen::Vector3d pos3d = Eigen::Vector3d(pos(0), pos(1), 0.0);
            // if (!rog_map->insideLocalMap(pos3d)) { distance = 1e+10; return; }

            // get the distance from the local map
            Eigen::Vector3d grad;
            rog_map->esdf_map_->getValueGrad2d(pos3d, distance, grad);
            return;
        }

        if (!isInMap2d(pos))
        {
            distance = 1e+10;
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;

        Eigen::Vector2i idx;
        posToIndex2d(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos2d(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;

        double values[2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex2d(current_idx);
                values[x][y] = esdf_buffer_2d[toAddress2d(current_idx)];
            }
        
        // value & grad
        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1 - diff[1]) + v1 * diff[1];

        return;
    }

    inline void GridMap::getDistance3d(const Eigen::Vector3d& pos, double& distance)
    {
        // MARK: 3d distance
        if (use_rog) {
            // if (!rog_map->insideLocalMap(pos)) 
            if (false)
            { 
                std::cout << "Point is not inside the local map" << pos.transpose() << std::endl;
                throw std::runtime_error("Point is not inside the local map");
                distance = 1e+10; return; }

            Eigen::Vector3d grad;
            rog_map->esdf_map_->evaluateEDT(pos, distance);
            // rog_map->esdf_map_->getCriticalValueGrad(pos, distance, grad);
            return;
        }


        if (!isInMap3d(pos))
        {
            distance = 1e+10;
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos.array() - 0.5 * resolution;

        Eigen::Vector3i idx;
        posToIndex3d(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos3d(idx, idx_pos);

        Eigen::Vector3d diff = (pos - idx_pos) * resolution_inv;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    boundIndex3d(current_idx);
                    values[x][y][z] = esdf_buffer_3d[toAddress3d(current_idx)];
                }
        
        // value & grad
        double v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        double v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        double v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        double v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        double v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        double v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        distance = v0 * (1.0 - diff[2]) + v1 * diff[2];

        return;
    }

    inline void GridMap::getDisWithGradI2d(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad, 
                                         bool inflate, bool critical)
    {
        if (use_rog)
        {
            double inflation = inflate ? moma_param.chassis_colli_radius : 0.0;
            Eigen::Vector3d pos3d(pos(0), pos(1), 0.0);
            // if (!rog_map->insideLocalMap(pos3d))
            if (false)
            {
                // throw std::runtime_error("Point is not inside the local map");
                distance = 0.0;
                grad.setZero();
                return;
            }
            else if (critical)
            {
                Eigen::Vector3d grad3d;
                rog_map->esdf_map_->getCriticalValueGrad(pos3d, distance, grad3d);
                distance -= inflation;
                grad = grad3d.head(2);
                return;
            } else {
                Eigen::Vector3d grad3d;
                rog_map->esdf_map_->getValueGrad2d(pos3d, distance, grad3d);
                distance -= inflation;
                grad = grad3d.head(2);
                return;
            }
        }

        if (!isInMap2d(pos))
        {
            distance = 0.0;
            grad.setZero();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;

        Eigen::Vector2i idx;
        posToIndex2d(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos2d(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;

        double values[2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex2d(current_idx);
                if (critical)
                    values[x][y] = esdf_buffer_2d_critical[toAddress2d(current_idx)];
                else if (inflate)
                    values[x][y] = esdf_buffer_2d_inflate[toAddress2d(current_idx)];
                else
                    values[x][y] = esdf_buffer_2d[toAddress2d(current_idx)];
            }
        
        // value & grad
        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1 - diff[1]) + v1 * diff[1];
        grad(1) = (v1 - v0) * resolution_inv;
        grad(0) = (1 - diff[1]) * (values[1][0] - values[0][0]);
        grad(0) += diff[1] * (values[1][1] - values[0][1]);
        grad(0) *= resolution_inv;

        return;
    }

    inline void GridMap::getDisWithGradI3d(const Eigen::Vector3d& pos, double& distance, Eigen::Vector3d& grad)
    {
        if (use_rog)
        {
            // if (!rog_map->insideLocalMap(pos))
            if (false)
            {   
                std:: cout << "Point is not inside the local map" << pos.transpose() << std::endl;
                throw std::runtime_error("Point is not inside the local map");
                distance = 0.0;
                grad.setZero();
                return;
            }
            else
            {
                rog_map->esdf_map_->evaluateEDT(pos, distance);
                rog_map->esdf_map_->evaluateFirstGrad(pos, grad);
                return;
            }
        }

        if (!isInMap3d(pos))
        {
            distance = 0.0;
            grad.setZero();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector3d pos_m = pos.array() - 0.5 * resolution;

        Eigen::Vector3i idx;
        posToIndex3d(pos_m, idx);

        Eigen::Vector3d idx_pos;
        indexToPos3d(idx, idx_pos);

        Eigen::Vector3d diff = (pos - idx_pos) * resolution_inv;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    boundIndex3d(current_idx);
                    values[x][y][z] = esdf_buffer_3d[toAddress3d(current_idx)];
                }
        
        // value & grad
        double v00 = values[0][0][0] * (1 - diff[0]) + values[1][0][0] * diff[0];
        double v01 = values[0][0][1] * (1 - diff[0]) + values[1][0][1] * diff[0];
        double v10 = values[0][1][0] * (1 - diff[0]) + values[1][1][0] * diff[0];
        double v11 = values[0][1][1] * (1 - diff[0]) + values[1][1][1] * diff[0];
        double v0 = v00 * (1 - diff[1]) + v10 * diff[1];
        double v1 = v01 * (1 - diff[1]) + v11 * diff[1];
        distance = v0 * (1.0 - diff[2]) + v1 * diff[2];
        grad(2) = (v1 - v0) * resolution_inv;
        grad(1) = ((v10 - v00) * (1.0 - diff[2]) + (v11 - v01) * diff[2]) * resolution_inv;
        grad(0) = (1.0 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad(0) += (1.0 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad(0) += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad(0) += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad(0) *= resolution_inv;

        return;
    }

    inline bool GridMap::isCollision2d(const Eigen::Vector2d& pos, double threshold)
    {
        if (use_rog)
        {
            Eigen::Vector3d pos3d(pos(0), pos(1), 0.0);
            // if (rog_map->insideLocalMap(pos3d))
            if(true)
            {
                double dist;
                Eigen::Vector3d grad;
                rog_map->esdf_map_->getValueGrad2d(pos3d, dist, grad);
                return dist < threshold;
            }
            else
                return true;
        }

        if (isInMap2d(pos))
        {
            double dist;
            getDistance2d(pos, dist);
            return dist < threshold;
        }
        else
            return true;
    }

    inline bool GridMap::isCollisionIndx2d(int x, int y, double threshold)
    {
        if (use_rog)
        {
            Eigen::Vector3i id_l(x, y, 0);
            id_l -= rog_map->esdf_map_->getHalfMapSize(); //! half map size
            id_l(2) = 0;

            Eigen::Vector3d pos3d;
            rog_map->esdf_map_->localIndexToPos(id_l, pos3d);

            double dist;
            Eigen::Vector3d grad;
            rog_map->esdf_map_->getValueGrad2d(pos3d, dist, grad);

            return dist < threshold;
        }
        return esdf_buffer_2d[toAddress2d(x, y)] < threshold;
    }

    inline bool GridMap::isCollisionGrid2d(const Eigen::Vector2d& pos, double threshold)
    {
        Eigen::Vector2i idx;
        posToIndex2d(pos, idx);
        return isCollisionIndx2d(idx.x(), idx.y());
    }

    inline bool GridMap::isLineCollisionGrid2d(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double threshold)
    {

        if (use_rog)
        {
            return !rog_map->esdf_map_->isLineFree2d(p1, p2, threshold);
        }

        std::vector<Eigen::Vector2i> line;
        Eigen::Vector2i start, end;
        posToIndex2d(p1, start);
        posToIndex2d(p2, end);
    
        int dx = abs(end.x() - start.x());
        int dy = abs(end.y() - start.y());
        int sx = (start.x() < end.x()) ? 1 : -1;
        int sy = (start.y() < end.y()) ? 1 : -1;
        int err = dx - dy;

        int x0 = start.x();
        int y0 = start.y();

        bool occ = false;
        while (true) 
        {
            line.emplace_back(x0, y0);
            if (isCollisionIndx2d(x0, y0, threshold))
            {
                occ = true;
                break;
            }
            if (x0 == end.x() && y0 == end.y()) break;
            int e2 = 2 * err;
            if (e2 > -dy) 
            {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) 
            {
                err += dx;
                y0 += sy;
            }
        }

        return occ;
    }

    inline bool GridMap::isWholeBodyCollision(const Eigen::VectorXd& state)
    {
        // joint position limit
        for (size_t i = 0; i < moma_param.dof_num; i++)
            if (state(i+3) > moma_param.joint_pos_limit_max[i] || state(i+3) < moma_param.joint_pos_limit_min[i])
                return true;

        // chassis with obstacles
        if (isCollision2d(state.head(2), moma_param.chassis_colli_radius))
            return true;

        std::vector<Eigen::Vector4d> colli_pts = moma_param.getColliPts(state);
        for (size_t i = 0; i < colli_pts.size(); i++)
        {
            // manipulators with obstacles
            if (isCollision3d(colli_pts[i].head(3), colli_pts[i](3))) 
            {
                return true;
            }

            // self collision: chassis with manipulators
            if (i>2 && colli_pts[i](2) < moma_param.chassis_height + colli_pts[i](3) &&
                (colli_pts[i].head(2) - state.head(2)).norm() < moma_param.chassis_colli_radius + colli_pts[i](3))
                return true;

            // self collision: link with link
            for (size_t j=i+1; j<colli_pts.size(); j++)
            {
                if (i == j)
                    continue;
                double dist = (colli_pts[i].head(3) - colli_pts[j].head(3)).norm();
                if (dist < colli_pts[i][3] + colli_pts[j][3] && moma_param.collision_matrix(i, j) == -1.0)
                    return true;
            }
        }

        return false;
    }

    inline bool GridMap::isWholeBodyCollision(const Eigen::VectorXd& state, int& coll_type) {
        // joint position limit
        for (size_t i = 0; i < moma_param.dof_num; i++)
            if (state(i+3) > moma_param.joint_pos_limit_max[i] || state(i+3) < moma_param.joint_pos_limit_min[i]) {
                coll_type = 4;
                return true;
            }

        // chassis with obstacles
        if (isCollision2d(state.head(2), moma_param.chassis_colli_radius)) {
            coll_type = 0;
            return true;
        }

        std::vector<Eigen::Vector4d> colli_pts = moma_param.getColliPts(state);
        for (size_t i = 0; i < colli_pts.size(); i++)
        {
            // manipulators with obstacles
            if (isCollision3d(colli_pts[i].head(3), colli_pts[i](3))) 
            {
                coll_type = 1;
                return true;
            }

            // self collision: chassis with manipulators
            if (i>2 && colli_pts[i](2) < moma_param.chassis_height + colli_pts[i](3) &&
                (colli_pts[i].head(2) - state.head(2)).norm() < moma_param.chassis_colli_radius + colli_pts[i](3)) {
                    coll_type = 2;
                    return true;
                }

            // self collision: link with link
            for (size_t j=i+1; j<colli_pts.size(); j++)
            {
                if (i == j)
                    continue;
                double dist = (colli_pts[i].head(3) - colli_pts[j].head(3)).norm();
                if (dist < colli_pts[i][3] + colli_pts[j][3] && moma_param.collision_matrix(i, j) == -1.0) {
                    coll_type = 3;
                    return true;
                }
            }
        }
        coll_type = -1;
        return false;
    }

    inline bool GridMap::isCollision3d(const Eigen::Vector3d& pos, double threshold)
    {
        if (use_rog)
        {
            // if (rog_map->insideLocalMap(pos))
            if(true)
            {
                double dist;
                Eigen::Vector3d grad;
                rog_map->esdf_map_->evaluateEDT(pos, dist);
                return dist < threshold;
            }
            else{
                std::cout << pos.transpose() << " is not inside the local map" << std::endl;
                throw std::runtime_error("Point is not inside the local map");
            }
        }

        if (isInMap3d(pos))
        {
            double dist;
            getDistance3d(pos, dist);
            return dist < threshold;
        }
        else
            return true;
    }

    inline void GridMap::boundIndex2d(Eigen::Vector2i& id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));

        return;
    }

    inline void GridMap::boundIndex3d(Eigen::Vector3i& id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));
        id(2) = max(min(id(2), max_idx(2)), min_idx(2));

        return;
    }

    inline void GridMap::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
    {
        if (use_rog)
        {
            Eigen::Vector3d pos3d(pos(0), pos(1), 0.0);
            Eigen::Vector3i id_g, id_l;
            rog_map->esdf_map_->posToGlobalIndex(pos3d, id_g);
            rog_map->esdf_map_->globalIndexToLocalIndex(id_g, id_l);
            id_l += rog_map->esdf_map_->getHalfMapSize(); //! half map size
            id = id_l.head(2);
            return;
        }
        id(0) = floor((pos(0) - map_origin(0)) * resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * resolution_inv);
        return;
    }

    inline void GridMap::posToIndex3d(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
    {
        id(0) = floor((pos(0) - map_origin(0)) * resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * resolution_inv);
        id(2) = floor((pos(2) - map_origin(2)) * resolution_inv);
        return;
    }

    inline void GridMap::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
    {
        if (use_rog)
        {
            Eigen::Vector3i id_l_corner(id(0), id(1), 0.0);
            id_l_corner -= rog_map->esdf_map_->getHalfMapSize(); //! half_map_size
            Eigen::Vector3d pos3d;
            rog_map->esdf_map_->localIndexToPos(id_l_corner, pos3d);
            pos = pos3d.head(2);
            return;
        }
        pos(0) = (id(0) + 0.5) * resolution  + map_origin(0);
        pos(1) = (id(1) + 0.5) * resolution  + map_origin(1);
        return;
    }

    inline void GridMap::indexToPos3d(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
    {
        if (use_rog) {
            // return rog_map->esdf_map_-> localIndexToPos(id, pos);
            return rog_map->esdf_map_-> globalIndexToPos(id, pos);
        }

        pos(0) = (id(0) + 0.5) * resolution  + map_origin(0);
        pos(1) = (id(1) + 0.5) * resolution  + map_origin(1);
        pos(2) = (id(2) + 0.5) * resolution  + map_origin(2);
        return;
    }

    inline int GridMap::toAddress2d(const Eigen::Vector2i& id) 
    {
        return id(0) * voxel_num(1) + id(1);
    }

    inline int GridMap::toAddress2d(const int& x, const int& y) 
    {
        return x * voxel_num(1) + y;
    }

    inline int GridMap::toAddress3d(const Eigen::Vector3i& id)
    {
        return id(0) * voxel_num(1) * voxel_num(2) + id(1) * voxel_num(2) + id(2);
    }

    inline int GridMap::toAddress3d(const int& x, const int& y, const int& z)
    {
        return x * voxel_num(1) * voxel_num(2) + y * voxel_num(2) + z;
    }
    
    inline bool GridMap::isInMap2d(const Eigen::Vector2d& pos) 
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 || \
            pos(1) > max_boundary(1) - 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isInMap2d(const Eigen::Vector2i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isInMap3d(const Eigen::Vector3d& pos)
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4 || \
            pos(2) < min_boundary(2) + 1e-4    ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 || \
            pos(1) > max_boundary(1) - 1e-4 || \
            pos(2) > max_boundary(2) - 1e-4    ) 
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isInMap3d(const Eigen::Vector3i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1 || \
            idx(2) > voxel_num(2) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline double GridMap::getDistCoarse2d(const Eigen::Vector2d& pos, bool critical)
    {
        if (use_rog) {
            Eigen::Vector3d pos3d(pos(0), pos(1), 0.0);
            Eigen::Vector3d grad;
            double distance;
            if (critical) {
                rog_map->esdf_map_->getCriticalValueGrad(pos3d, distance, grad);
                distance -= moma_param.chassis_colli_radius;
                return distance;
            }
            else {
                rog_map->esdf_map_->getValueGrad2d(pos3d, distance, grad);
                distance -= moma_param.chassis_colli_radius;
                return distance;
            }
        }
        Eigen::Vector2i id;
        posToIndex2d(pos, id);
        boundIndex2d(id);
        if (critical)
            return esdf_buffer_2d_critical[toAddress2d(id)];
        else
            return esdf_buffer_2d_inflate[toAddress2d(id)];

        return esdf_buffer_2d[toAddress2d(id)];
    }

    inline double GridMap::getDistCoarse2i(const Eigen::Vector2i& id, bool critical)
    {
        if (use_rog) {
            double distance;
            Eigen::Vector3i id3(id(0), id(1), 0.0);
            Eigen::Vector3d pos3d, grad;
            rog_map->esdf_map_->globalIndexToPos(id3, pos3d);
            if (critical) {
                rog_map->esdf_map_->getCriticalValueGrad(pos3d, distance, grad);
                distance -= moma_param.chassis_colli_radius;
                return distance;
            } else {
                rog_map->esdf_map_->getValueGrad2d(pos3d, distance, grad);
                distance -= moma_param.chassis_colli_radius;
                return distance;
            }
        }
        Eigen::Vector2i id1 = id;
        boundIndex2d(id1);
        if (critical)
            return esdf_buffer_2d_critical[toAddress2d(id1)];
        else
            return esdf_buffer_2d_inflate[toAddress2d(id1)];

        return esdf_buffer_2d[toAddress2d(id1)];
    }

    inline bool GridMap::mapReady()
    {
        return map_ready;
    }
}
