#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <time.h>
#include <random>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>

#include "map/grid_map.h"

namespace nmoma_planner
{
    struct RRTNode;
    typedef RRTNode* RRTNodePtr;

    struct RRTNode
    {
        enum NodeState
        {
            INIT,
            EXPANDED,
            IN_TREE,
            IN_ANTI_TREE
        };

        int mc_idx = -1;
        string key;
        double cost;
        NodeState node_state = INIT;
        Eigen::VectorXd robo_state;  // xytheta, q1-q7
        RRTNodePtr parent = nullptr;
        std::map<string, RRTNodePtr> children;
    };

    class BiRRTs
    {
        private:
            // params
            int state_dim;
            double rewire_time_range;
            double steer_time;
            double max_time;
            double check_colli_res;
            MomaParam moma_param;
            Eigen::VectorXd sample_min;
            Eigen::VectorXd sample_max;
            
            // datas
            bool connected = false;
            double c_min = 0.0;
            double c_max = 1.0e+6;
            GridMap::Ptr grid_map;
            RRTNodePtr goal_node = nullptr;
            Eigen::VectorXd sample_start;
            Eigen::VectorXd sample_end;
            double sample_start_radius = 0.0;
            double sample_end_radius = 0.0;
            std::map<string, RRTNodePtr> node_pool;
            ompl::base::StateSpacePtr se2_geop;

            // random
            std::mt19937 rand_gen;
            std::uniform_real_distribution<double> uniform_sampler;
            std::normal_distribution<double> normal_sampler;

            inline void updateCosts(RRTNodePtr& node);
            inline double estTime(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2);
            inline double estHeuristic(RRTNodePtr& node_1, RRTNodePtr& node_2);
            inline double estHeuristic(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2);
            inline string getKey(const Eigen::VectorXd &state);
            inline RRTNodePtr genNodeFromState(const Eigen::VectorXd &state);
            inline Eigen::VectorXd sampleState();
            inline RRTNodePtr getNearestNode(const Eigen::VectorXd &state, bool anti);
            inline void linkNode(RRTNodePtr& parent, RRTNodePtr& child);
            inline void mergeTree(RRTNodePtr& s1, RRTNodePtr& s2);
            inline bool connectCollision(const Eigen::VectorXd& cur_state, const Eigen::VectorXd& next_state);

            RRTNodePtr steer(RRTNodePtr &node, const Eigen::VectorXd &target_state);
            void rewire(RRTNodePtr q_new);
        
        public:
            BiRRTs(GridMap::Ptr grid_map_) :
                    grid_map(grid_map_),
                    rand_gen(std::random_device{}()), 
                    uniform_sampler(0.0, 1.0), 
                    normal_sampler(0.0, 0.1) {}

            ~BiRRTs() 
            {
                for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
                    delete it->second;
                node_pool.clear();
            }

            inline void init(ros::NodeHandle& nh);
            inline void reset(const vector<Eigen::VectorXd>& start_states, const vector<Eigen::VectorXd>& end_states);
            inline void reset(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state);

            bool plan(const Eigen::VectorXd& start, const Eigen::VectorXd &end,
                      std::vector<Eigen::VectorXd>& path, std::vector<double>& time_list);
            bool plan(const std::vector<Eigen::VectorXd>& starts, const std::vector<Eigen::VectorXd>& ends,
                      const std::vector<double>& start_costs, const std::vector<double>& end_costs,
                      const std::vector<int>& start_mcidx, const std::vector<int>& end_mcidx,
                      std::vector<Eigen::VectorXd>& path, std::vector<double>& time_list, std::pair<int, int>& layer);
            
            typedef std::shared_ptr<BiRRTs> Ptr;
    };

    inline void BiRRTs::init(ros::NodeHandle& nh)
    {
        nh.getParam("birrts/rewire_time_range", rewire_time_range);
        nh.getParam("birrts/steer_time", steer_time);
        nh.getParam("birrts/max_time", max_time);
        nh.getParam("birrts/check_colli_res", check_colli_res);

        state_dim = 3 + moma_param.dof_num;
        sample_min.resize(state_dim);
        sample_max.resize(state_dim);
        sample_min.head(2) = grid_map->getMinBound();
        sample_max.head(2) = grid_map->getMaxBound();
        sample_min(2) = -M_PI;
        sample_max(2) = M_PI;
        sample_min.tail(moma_param.dof_num) = moma_param.joint_pos_limit_min;
        sample_max.tail(moma_param.dof_num) = moma_param.joint_pos_limit_max;
        se2_geop =std::make_shared<ompl::base::DubinsStateSpace>(1.0e-2);
        // se2_geop =std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0e-2);

        return;
    }

    inline void BiRRTs::reset(const vector<Eigen::VectorXd>& start_states, const vector<Eigen::VectorXd>& end_states)
    {
        connected = false;
        goal_node = nullptr;
        for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
            delete it->second;
        node_pool.clear();
        c_max = 1.0e+6;
        sample_start_radius = 0.0;
        sample_start = start_states[0];
        if (start_states.size() > 1)
        {
            for(size_t i = 0; i < start_states.size(); ++i)
            {
                for(size_t j = i + 1; j < start_states.size(); ++j)
                {
                    double dist = (start_states[i] - start_states[j]).norm();
                    if(dist > sample_start_radius)
                    {
                        sample_start = (start_states[i] + start_states[j]) / 2.0;
                        sample_start_radius = dist;
                    }
                }
            }
        }
        sample_end_radius = 0.0;
        sample_end = end_states[0];
        if (end_states.size() > 1)
        {
            for(size_t i = 0; i < end_states.size(); ++i)
            {
                for(size_t j = i + 1; j < end_states.size(); ++j)
                {
                    double dist = (end_states[i] - end_states[j]).norm();
                    if(dist > sample_end_radius)
                    {
                        sample_end = (end_states[i] + end_states[j]) / 2.0;
                        sample_end_radius = dist;
                    }
                }
            }
        }
        c_min = estHeuristic(sample_start, sample_end);
        return;
    }

    inline void BiRRTs::reset(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state)
    {
        connected = false;
        goal_node = nullptr;
        for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
            delete it->second;
        node_pool.clear();
        c_max = 1.0e+6;
        c_min = estHeuristic(start_state, end_state);
        sample_start = start_state;
        sample_end = end_state;
        sample_start_radius = 0.0;
        sample_end_radius = 0.0;
        return;
    }

    inline void BiRRTs::updateCosts(RRTNodePtr& node)
    {
        double now_cost = node->parent->cost + estHeuristic(node->parent->robo_state, node->robo_state);
        if(node->cost == now_cost)
            return;
        node->cost = now_cost;
        for(auto it = node->children.begin(); it != node->children.end(); ++it)
            updateCosts(it->second);
    }

    inline double BiRRTs::estTime(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)
    {
        // reeds shepp path get car check num
        ompl::base::ScopedState<> from(se2_geop), to(se2_geop), s(se2_geop);
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        double time = se2_geop->distance(from(), to()) / moma_param.max_v;
        
        Eigen::VectorXd delta_theta = state2.tail(moma_param.dof_num) - state1.tail(moma_param.dof_num);
        for (size_t i = 0; i < moma_param.dof_num; ++i)
            time = max(time, fabs(delta_theta(i)) / moma_param.joint_vel_limit(i));

        return time;
    }

    inline double BiRRTs::estHeuristic(RRTNodePtr& node_1, RRTNodePtr& node_2)
    {
        return estHeuristic(node_1->robo_state, node_2->robo_state);
    }

    inline double BiRRTs::estHeuristic(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)
    {
        Eigen::VectorXd diff(moma_param.dof_num+3);
        double dist2 = (state1.head(2) - state2.head(2)).squaredNorm();
        double dtheta = fabs(state1(2) - state2(2));
        dtheta = dtheta > M_PI ? 2.0 * M_PI - dtheta : dtheta;
        double dq2 = (state1.tail(moma_param.dof_num) - state2.tail(moma_param.dof_num)).squaredNorm();

        return sqrt(dist2 + dq2 + 10.0 * (dtheta*dtheta));
    }

    inline string BiRRTs::getKey(const Eigen::VectorXd &state)
    {
        string ret;
        for(int i = 0; i < state.size(); ++i)
            ret += std::to_string((int) (round(state(i) * 100.0)));
        return ret;
    }

    inline RRTNodePtr BiRRTs::genNodeFromState(const Eigen::VectorXd &state)
    {
        string key = getKey(state);
        if (!node_pool.empty())
        {
            auto it = node_pool.find(key);
            if(it != node_pool.end())
                return it->second;
        }
        RRTNodePtr node = new RRTNode;
        node->node_state = RRTNode::EXPANDED;
        node->robo_state = state;
        node->key = key;
        node_pool.insert(std::make_pair(key, node));
        return node;
    }

    inline Eigen::VectorXd BiRRTs::sampleState()
    {
        Eigen::VectorXd sample_state(state_dim);
        if(c_max < 1.0e+5)
        {
            sample_start(2) = -M_PI;
            sample_end(2) = M_PI;
            Eigen::VectorXd sample_center = (sample_start + sample_end) / 2.0;
            Eigen::VectorXd a1 = (sample_end - sample_start).normalized();
            Eigen::MatrixXd M = a1 * Eigen::VectorXd::Ones(state_dim).transpose();
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
            Eigen::MatrixXd V = svd.matrixV();
            Eigen::MatrixXd U = svd.matrixU();
            Eigen::MatrixXd C_middle(state_dim, state_dim);
            C_middle.setZero();
            for(int i = 0; i < state_dim - 1; ++i)
                C_middle(i, i) = 1.0;
            C_middle(state_dim-1, state_dim-1) = U.determinant() * V.determinant();

            Eigen::MatrixXd C = U * C_middle * V.transpose();
            Eigen::MatrixXd L(state_dim, state_dim);
            L.setZero();
            double new_c_max = c_max + (sample_start_radius + sample_end_radius) / 2.0;
            for(int i = 1; i < state_dim; ++i)
                L(i, i) = sqrt(new_c_max * new_c_max - c_min * c_min) / 2;
            L(0, 0) = new_c_max / 2.0;

            // Uniform sampling within unit sphere
            for(int i = 0; i < state_dim; ++i)
                sample_state(i) = normal_sampler(rand_gen);
            sample_state.normalize();
            double r = pow(uniform_sampler(rand_gen), 1.0/state_dim);
            sample_state *= r;
            sample_state = C * L * sample_state + sample_center;
            for(int i = 0; i < state_dim; ++i)
                sample_state[i] = min(max(sample_min[i], sample_state[i]), sample_max[i]);
        }
        else
        {
            for(int i = 0; i < state_dim; ++i)
                sample_state(i) = sample_min[i] + (sample_max[i] - sample_min[i]) * uniform_sampler(rand_gen);
        }
        return sample_state;
    }

    inline RRTNodePtr BiRRTs::getNearestNode(const Eigen::VectorXd &state, bool anti)
    {
        RRTNodePtr q_near = nullptr;
        double min_dis = 1.0e6;
        for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
        {
            RRTNodePtr node = it->second;
            if((!anti && node->node_state == RRTNode::IN_TREE) || 
                (anti && node->node_state == RRTNode::IN_ANTI_TREE))
            {
                if ((node->robo_state - state).norm() > 1.0e-2 &&
                    estHeuristic(node->robo_state, state) < min_dis)
                {
                    min_dis = estHeuristic(node->robo_state, state);
                    q_near = node;
                }
            }
        }
        return q_near;
    }

    inline void BiRRTs::linkNode(RRTNodePtr& parent, RRTNodePtr& child)
    {
        RRTNodePtr pre_parent = child->parent;
        if (pre_parent == parent)
            return;
        if (pre_parent != nullptr)
            pre_parent->children.erase(child->key);
        child->parent = parent;    
        parent->children.insert(std::make_pair(child->key, child));
        updateCosts(child);
        return;
    }

    inline void BiRRTs::mergeTree(RRTNodePtr& s1, RRTNodePtr& s2)
    {
        RRTNodePtr q1, q2, q_temp;
        if(s1->node_state == RRTNode::IN_TREE)
        {
            q1 = s1;
            q2 = s2;
        }
        else
        {
            q1 = s2;
            q2 = s1;
        }

        std::vector<RRTNodePtr> node_list;
        q_temp = q2;
        while(q_temp != nullptr)
        {
            node_list.push_back(q_temp);
            q_temp->node_state = RRTNode::IN_TREE;
            q_temp->children.clear();
            q_temp = q_temp->parent;
        }
        linkNode(q1, node_list[0]);
        for(size_t i = 1; i < node_list.size(); ++i)
            linkNode(node_list[i-1], node_list[i]);
        goal_node = node_list.back();
        return;
    }

    inline bool BiRRTs::connectCollision(const Eigen::VectorXd& cur_state, 
                                         const Eigen::VectorXd& next_state)
    {
        // reeds shepp path get car check num
        ompl::base::ScopedState<> from(se2_geop), to(se2_geop), s(se2_geop);
        from[0] = cur_state[0]; from[1] = cur_state[1]; from[2] = cur_state[2];
        to[0] = next_state[0]; to[1] = next_state[1]; to[2] = next_state[2];
        int check_num_car = ceil(se2_geop->distance(from(), to()) / check_colli_res);
        
        // linear interpolation get manipulator check num
        Eigen::VectorXd delta_theta = next_state.tail(moma_param.dof_num) - cur_state.tail(moma_param.dof_num);
        int check_num_theta = ceil(delta_theta.lpNorm<Eigen::Infinity>() / check_colli_res);

        // check collision
        double piece_num_temp = 1.0 * std::max(std::max(check_num_car, check_num_theta), 3);
        for(int i = 0; i < piece_num_temp; ++i)
        {
            Eigen::VectorXd temp_state(state_dim);
            double temp_i = 1.0 * i / piece_num_temp;
            se2_geop->interpolate(from(), to(), temp_i, s());
            auto reals = s.reals();
            temp_state[0] = reals[0];
            temp_state[1] = reals[1];
            temp_state[2] = reals[2];
            temp_state.tail(moma_param.dof_num) = cur_state.tail(moma_param.dof_num) + delta_theta * temp_i;
            if (grid_map->isWholeBodyCollision(temp_state))
                return true;
        }

        return false;
    }
}