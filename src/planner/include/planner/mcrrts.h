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
    struct MCRRTNode;
    typedef MCRRTNode* MCRRTNodePtr;
    typedef std::pair<int, Eigen::VectorXd> MCState;

    struct MCRRTNode
    {
        enum NodeState
        {
            INIT,
            EXPANDED,
            IN_TREE,
            IN_ANTI_TREE
        };

        string key;
        double cost;
        NodeState node_state = INIT;
        MCState robo_state;  // idx, q1-q7
        MCRRTNodePtr parent = nullptr;
        std::map<string, MCRRTNodePtr> children;
    };

    class MCRRTs
    {
        private:
            // params
            int state_dim;
            double goal_sample_rate;
            double max_time;
            double check_colli_res;
            MomaParam moma_param;
            Eigen::VectorXd sample_min;
            Eigen::VectorXd sample_max;
            
            // datas
            int near_min_idx = 1<<20;
            int near_max_idx = 0;
            bool connected = false;
            double c_max = 1.0e+6;
            GridMap::Ptr grid_map;
            MCRRTNodePtr goal_node = nullptr;
            std::vector<Eigen::Vector4d> car_path;
            std::map<string, MCRRTNodePtr> node_pool;
            ompl::base::StateSpacePtr reeds_shepp;

            // random
            std::mt19937 rand_gen;
            std::uniform_real_distribution<double> uniform_sampler;

            inline void updateMinMaxIdx(MCRRTNodePtr& node);
            inline void updateCosts(MCRRTNodePtr& node);
            inline double estHeuristic(const MCRRTNodePtr& node_1, const MCRRTNodePtr& node_2);
            inline double estHeuristic(const MCState& state_1, const MCState& state_2);
            inline string getKey(const MCState &state);
            inline MCRRTNodePtr genNodeFromState(const MCState &state);
            inline MCState sampleState();
            inline MCRRTNodePtr getNearestNode(const MCState &state, bool anti);
            inline void linkNode(MCRRTNodePtr& parent, MCRRTNodePtr& child);
            inline void mergeTree(MCRRTNodePtr& s1, MCRRTNodePtr& s2);
            inline bool feasibleCheck(const MCState& cur_tate, const MCState& next_state);
            inline bool connectCollision(const MCState& cur_state, const MCState& next_state);

            MCRRTNodePtr steer(MCRRTNodePtr &node, const MCState &target_state);
            void rewire(MCRRTNodePtr q_new);
        
        public:
            MCRRTs(GridMap::Ptr grid_map_) :
                    grid_map(grid_map_),
                    rand_gen(std::random_device{}()), 
                    uniform_sampler(0.0, 1.0) {}

            ~MCRRTs() 
            {
                for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
                    delete it->second;
                node_pool.clear();
            }

            inline void init(ros::NodeHandle& nh);
            inline void reset(const std::vector<Eigen::Vector4d>& path);

            bool plan(const Eigen::VectorXd& start, const Eigen::VectorXd &end,
                      const std::vector<Eigen::Vector4d>& path, std::vector<Eigen::VectorXd>& wb_path);
            void getHoleNodes(std::vector<Eigen::VectorXd>& starts, std::vector<Eigen::VectorXd>& ends,
                              std::vector<double>& start_costs, std::vector<double>& end_costs,
                              std::vector<int>& start_mcidx, std::vector<int>& end_mcidx);
            void fillHole(const std::vector<Eigen::VectorXd>& hole_path, std::pair<int, int>& layer,
                          std::vector<Eigen::VectorXd>& wb_path);
            
            typedef std::shared_ptr<MCRRTs> Ptr;
            typedef std::unique_ptr<MCRRTs> UniPtr;
    };

    inline void MCRRTs::init(ros::NodeHandle& nh)
    {
        nh.getParam("mcrrts/max_time", max_time);
        nh.getParam("mcrrts/goal_sample_rate", goal_sample_rate);
        nh.getParam("mcrrts/check_colli_res", check_colli_res);

        state_dim = moma_param.dof_num;
        sample_min = moma_param.joint_pos_limit_min;
        sample_max = moma_param.joint_pos_limit_max;
        reeds_shepp =std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0e-2);

        return;
    }

    inline void MCRRTs::reset(const std::vector<Eigen::Vector4d>& path)
    {
        connected = false;
        goal_node = nullptr;
        for(auto it = node_pool.begin(); it != node_pool.end(); ++it)
            delete it->second;
        node_pool.clear();
        c_max = 1.0e+6;
        car_path = path;
        near_min_idx = car_path.size() - 1;
        near_max_idx = 0;
        return;
    }

    inline void MCRRTs::updateMinMaxIdx(MCRRTNodePtr& node)
    {
        if (node->node_state == MCRRTNode::IN_TREE &&
            node->robo_state.first > near_max_idx)
            near_max_idx = node->robo_state.first;
        if (node->node_state == MCRRTNode::IN_ANTI_TREE &&
            node->robo_state.first < near_min_idx)
            near_min_idx = node->robo_state.first;
        return;
    }

    inline void MCRRTs::updateCosts(MCRRTNodePtr& node)
    {
        double now_cost = node->parent->cost + estHeuristic(node->parent, node);
        if(node->cost == now_cost)
            return;
        node->cost = now_cost;
        for(auto it = node->children.begin(); it != node->children.end(); ++it)
            updateCosts(it->second);
    }

    inline double MCRRTs::estHeuristic(const MCRRTNodePtr& node_1, const MCRRTNodePtr& node_2)
    {
        return estHeuristic(node_1->robo_state, node_2->robo_state);
    }

    inline double MCRRTs::estHeuristic(const MCState& state_1, const MCState& state_2)
    {
        double time = 0.0;
        int min_idx = (state_1.first < state_2.first) ? state_1.first : state_2.first;
        int max_idx = (state_1.first > state_2.first) ? state_1.first : state_2.first;
        for (int i = min_idx; i < max_idx; ++i)
            time += car_path[i].w();

        return (state_1.second - state_2.second).lpNorm<1>() / time;
    }

    inline string MCRRTs::getKey(const MCState& state)
    {
        string res(1, state.first);
        for(int i = 0; i < state.second.size(); ++i)
            res += std::to_string((int) (round(state.second(i) * 100.0)));
        return res;
    }

    inline MCRRTNodePtr MCRRTs::genNodeFromState(const MCState &state)
    {
        string key = getKey(state);
        if (!node_pool.empty())
        {
            auto it = node_pool.find(key);
            if(it != node_pool.end())
                return it->second;
        }
        MCRRTNodePtr node = new MCRRTNode;
        node->node_state = MCRRTNode::EXPANDED;
        node->robo_state = state;
        node->key = key;
        node_pool.insert(std::make_pair(key, node));
        return node;
    }

    inline MCState MCRRTs::sampleState()
    {
        std::uniform_int_distribution<int> int_sampler(1, (int)car_path.size()-2);
        int idx = int_sampler(rand_gen);
        Eigen::VectorXd full_state(3+state_dim);
        full_state.head(3) = car_path[idx].head(3);
        Eigen::VectorXd sample_state(state_dim);
        ros::Time time_begin = ros::Time::now();
        while((ros::Time::now() - time_begin).toSec() < max_time && ros::ok())
        {
            for(int i = 0; i < state_dim; ++i)
                sample_state(i) = sample_min[i] + (sample_max[i] - sample_min[i]) * uniform_sampler(rand_gen);
            full_state.tail(state_dim) = sample_state;
            if (!grid_map->isWholeBodyCollision(full_state))
                break;
        }
        
        return std::make_pair(idx, sample_state);
    }

    inline MCRRTNodePtr MCRRTs::getNearestNode(const MCState &state, bool anti)
    {
        int near_layer = anti ? max(state.first+1, near_min_idx) : min(near_max_idx, state.first-1);
        MCRRTNodePtr q_near = nullptr;
        double min_dis = 1.0e12;
        for(auto it = node_pool.lower_bound(string(1, near_layer)); 
            it != node_pool.end() && it->second->robo_state.first == near_layer; ++it)
        {
            MCRRTNodePtr node = it->second;
            if((!anti && node->node_state == MCRRTNode::IN_TREE) || 
                (anti && node->node_state == MCRRTNode::IN_ANTI_TREE))
            {
                if (estHeuristic(node->robo_state, state) < min_dis)
                {
                    min_dis = estHeuristic(node->robo_state, state);
                    q_near = node;
                }
            }
        }
        return q_near;
    }

    inline void MCRRTs::linkNode(MCRRTNodePtr& parent, MCRRTNodePtr& child)
    {
        MCRRTNodePtr pre_parent = child->parent;
        if (pre_parent == parent)
            return;
        if (pre_parent != nullptr)
            pre_parent->children.erase(child->key);
        child->parent = parent;
        parent->children.insert(std::make_pair(child->key, child));
        updateCosts(child);
        return;
    }

    inline void MCRRTs::mergeTree(MCRRTNodePtr& s1, MCRRTNodePtr& s2)
    {
        MCRRTNodePtr q1, q2, q_temp;
        if(s1->node_state == MCRRTNode::IN_TREE)
        {
            q1 = s1;
            q2 = s2;
        }
        else
        {
            q1 = s2;
            q2 = s1;
        }

        std::vector<MCRRTNodePtr> node_list;
        q_temp = q2;
        while(q_temp != nullptr)
        {
            node_list.push_back(q_temp);
            q_temp->node_state = MCRRTNode::IN_TREE;
            q_temp->children.clear();
            q_temp = q_temp->parent;
        }
        linkNode(q1, node_list[0]);
        for(size_t i = 1; i < node_list.size(); ++i)
            linkNode(node_list[i-1], node_list[i]);
        goal_node = node_list.back();
        return;
    }

    inline bool MCRRTs::feasibleCheck(const MCState& state_1, const MCState& state_2)
    {
        double time = 0.0;
        int min_idx = (state_1.first < state_2.first) ? state_1.first : state_2.first;
        int max_idx = (state_1.first > state_2.first) ? state_1.first : state_2.first;
        for (int i = min_idx; i < max_idx; ++i)
            time += car_path[i].w();
        Eigen::VectorXd dif = state_2.second - state_1.second;
        for (int i=0; i<dif.size(); ++i)
            if (dif(i) > M_PI)
                dif(i) = 2.0 * M_PI - fabs(dif(i));
        Eigen::VectorXd vel = dif.cwiseAbs() / time;

        if ((moma_param.joint_vel_limit-vel).minCoeff() < 0.0)
            return false;
        return true;
    }

    inline bool MCRRTs::connectCollision(const MCState& cur_state_full, const MCState& next_state_full)
    {
        Eigen::VectorXd cur_state;
        cur_state.resize(3+state_dim);
        cur_state.head(3) = car_path[cur_state_full.first].head(3);
        cur_state.tail(state_dim) = cur_state_full.second;
        Eigen::VectorXd next_state;
        next_state.resize(3+state_dim);
        next_state.head(3) = car_path[next_state_full.first].head(3);
        next_state.tail(state_dim) = next_state_full.second;

        // reeds shepp path get car check num
        ompl::base::ScopedState<> from(reeds_shepp), to(reeds_shepp), s(reeds_shepp);
        from[0] = cur_state[0]; from[1] = cur_state[1]; from[2] = cur_state[2];
        to[0] = next_state[0]; to[1] = next_state[1]; to[2] = next_state[2];
        int check_num_car = ceil(reeds_shepp->distance(from(), to()) / check_colli_res);
        
        // linear interpolation get manipulator check num
        Eigen::VectorXd delta_theta = next_state.tail(moma_param.dof_num) - cur_state.tail(moma_param.dof_num);
        int check_num_theta = ceil(delta_theta.lpNorm<Eigen::Infinity>() / check_colli_res);

        // check collision
        double piece_num_temp = 1.0 * std::max(std::max(check_num_car, check_num_theta), 3);
        for(int i = 0; i < piece_num_temp; ++i)
        {
            Eigen::VectorXd temp_state(3+state_dim);
            double temp_i = 1.0 * i / piece_num_temp;
            reeds_shepp->interpolate(from(), to(), temp_i, s());
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