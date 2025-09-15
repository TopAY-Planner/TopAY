#pragma once

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <Eigen/Eigen>
#include <random>

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Path.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/PlannerDataStorage.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/SimpleSetup.h>

#include "map/grid_map.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace nmoma_planner
{

    class MomaStateSpace : public ob::RealVectorStateSpace
    {
        public:
            GridMap::Ptr grid_map;
            MomaParam moma_param;
            ob::StateSpacePtr reeds_shepp;

        public:
            MomaStateSpace(GridMap::Ptr grid_map_, MomaParam moma_param_) : 
                ob::RealVectorStateSpace(moma_param_.dof_num+3),
                grid_map(grid_map_),
                moma_param(moma_param_)
            {
                ob::RealVectorBounds bounds(moma_param.dof_num+3);
                bounds.setLow(0, grid_map->min_boundary.x());
                bounds.setHigh(0, grid_map->max_boundary.x());
                bounds.setLow(1, grid_map->min_boundary.y());
                bounds.setHigh(1, grid_map->max_boundary.y());
                bounds.setLow(2, -M_PI);
                bounds.setHigh(2, M_PI);
                for (size_t i = 0; i < moma_param.dof_num; i++)
                {
                    bounds.setLow(i+3, moma_param.joint_pos_limit_min[i]);
                    bounds.setHigh(i+3, moma_param.joint_pos_limit_max[i]);
                }
                setBounds(bounds);
                reeds_shepp =std::make_shared<ob::ReedsSheppStateSpace>(1.0e-2);
            }

            double distSO2(const double& theta1, const double& theta2) const
            {
                double dist_so2 = fabs(theta1 - theta2);
                dist_so2 = dist_so2 > M_PI ? 2.0 * M_PI - dist_so2 : dist_so2;
                return dist_so2;
            }

            double getInterpolatedSO2(const double& theta1, const double& theta2, const double& t) const
            {
                double diff_so2 = theta2 - theta1;
                double i_value = theta1 + diff_so2 * t;
                if (fabs(diff_so2) > M_PI)
                {
                    if (diff_so2 > 0.0)
                        diff_so2 = 2.0 * M_PI - diff_so2;
                    else
                        diff_so2 = -2.0 * M_PI - diff_so2;
                    double v = theta1 - diff_so2 * t;
                    // input states are within bounds, so the following check is sufficient
                    if (v > M_PI)
                        v -= 2.0 * M_PI;
                    else if (v < -M_PI)
                        v += 2.0 * M_PI;
                    i_value = v;
                }

                return i_value;
            }

            void enforceBounds(ob::State *state) const override
            {
                auto *rstate = static_cast<StateType *>(state);
                double so2 = fmod(rstate->values[2], 2.0 * M_PI);
                if (so2 < -M_PI)
                    so2 += 2.0 * M_PI;
                else if (so2 >= M_PI)
                    so2 -= 2.0 * M_PI;
                rstate->values[2] = so2;
                for (unsigned int i = 0; i < dimension_; ++i)
                {
                    if (rstate->values[i] > bounds_.high[i])
                        rstate->values[i] = bounds_.high[i];
                    else if (rstate->values[i] < bounds_.low[i])
                        rstate->values[i] = bounds_.low[i];
                }
            }

            // double distance(const ob::State *state1, const ob::State *state2) const override
            // {
            //     const auto *cstate1 = state1->as<StateType>();
            //     const auto *cstate2 = state2->as<StateType>();

            //     Eigen::VectorXd diff(moma_param.dof_num+3);
            //     for (size_t i = 0; i < moma_param.dof_num + 3; i++)
            //         diff(i) = (cstate2->values[i] - cstate1->values[i]);

            //     double theta_direct = atan2(diff[1], diff[0]);
            //     double theta_direct_inv = atan2(-diff[1], -diff[0]);
            //     double dist_so2 = min(distSO2(cstate1->values[2], theta_direct) + distSO2(cstate2->values[2], theta_direct), 
            //                             distSO2(cstate1->values[2], theta_direct_inv) + distSO2(cstate2->values[2], theta_direct_inv));
            //     double time = diff.head(2).norm() / moma_param.max_v + dist_so2 / moma_param.max_w;
            //     for (size_t i = 0; i < moma_param.dof_num; i++)
            //         time = max(time, fabs(cstate1->values[i+3] - cstate2->values[i+3]) / moma_param.joint_vel_limit(i));
            //     return time;
            // }

            double distance(const ob::State *state1, const ob::State *state2) const override
            {
                const auto *cstate1 = state1->as<StateType>();
                const auto *cstate2 = state2->as<StateType>();

                Eigen::VectorXd diff(moma_param.dof_num+3);
                for (size_t i = 0; i < moma_param.dof_num + 3; i++)
                    diff(i) = (cstate2->values[i] - cstate1->values[i]);

                double dist_r2 = diff.head(2).norm();
                double dist_so2 = distSO2(cstate1->values[2], cstate2->values[2]);
                if (dist_r2 > 1e-2)
                {
                    double theta_direct = atan2(diff[1], diff[0]);
                    dist_so2 = distSO2(cstate1->values[2], theta_direct) + distSO2(cstate2->values[2], theta_direct);
                }

                double time = dist_r2 / moma_param.max_v + dist_so2 / moma_param.max_w;
                for (size_t i = 0; i < moma_param.dof_num; i++)
                    time = max(time, fabs(cstate1->values[i+3] - cstate2->values[i+3]) / moma_param.joint_vel_limit(i));
                return time;
            }

            void interpolate(const ob::State *from, const ob::State *to, const double t,
                     ob::State *state) const override
            {
                const auto *fromt = from->as<StateType>();
                const auto *tot = to->as<StateType>();
                auto *statet = state->as<StateType>();

                if (t == 0.0)
                {
                    for (size_t i = 0; i < moma_param.dof_num + 3; i++)
                        statet->values[i] = fromt->values[i];
                    return;
                }
                else if (t == 1.0)
                {
                    for (size_t i = 0; i < moma_param.dof_num + 3; i++)
                        statet->values[i] = tot->values[i];
                    return;
                }

                Eigen::VectorXd diff(moma_param.dof_num+3);
                for (size_t i = 0; i < moma_param.dof_num + 3; i++)
                    diff(i) = (tot->values[i] - fromt->values[i]);

                double dist_so2 = distSO2(fromt->values[2], tot->values[2]);
                double dist_r2 = diff.head(2).norm();
                if (dist_r2 > 1e-2)
                {
                    double theta_direct = atan2(diff[1], diff[0]);
                    double dist_so2_start = distSO2(fromt->values[2], theta_direct);
                    double dist_so2_end = distSO2(tot->values[2], theta_direct);
                    dist_so2 = dist_so2_start + dist_so2_end;
                    double theta_mid = theta_direct;

                    double dist_start = dist_so2_start;
                    double dist_end = dist_so2_end;
                    double dist_theta = dist_so2;

                    double t_total = distance(from, to);
                    double chassis_time = dist_r2 / moma_param.max_v + dist_theta / moma_param.max_w;
                    double chassis_v = chassis_time * moma_param.max_v / t_total;
                    double chassis_w = chassis_time * moma_param.max_w / t_total;

                    if (dist_start > t * t_total * chassis_w)
                    {
                        statet->values[0] = fromt->values[0];
                        statet->values[1] = fromt->values[1];
                        statet->values[2] = getInterpolatedSO2(fromt->values[2], theta_mid, t * t_total * chassis_w / dist_start);
                    }
                    else if (dist_r2 > (t * t_total - dist_start / chassis_w) * chassis_v)
                    {
                        statet->values[0] = fromt->values[0] + diff(0) * (t * t_total - dist_start / chassis_w) * chassis_v / dist_r2;
                        statet->values[1] = fromt->values[1] + diff(1) * (t * t_total - dist_start / chassis_w) * chassis_v / dist_r2;
                        statet->values[2] = theta_mid;
                    }
                    else
                    {
                        double temp = t* t_total - dist_start / chassis_w - dist_r2 / chassis_v;
                        statet->values[0] = tot->values[0];
                        statet->values[1] = tot->values[1];
                        statet->values[2] = getInterpolatedSO2(theta_mid, tot->values[2], temp * chassis_w / dist_end);
                    }
                }
                else
                {
                    statet->values[0] = fromt->values[0] + diff[0] * t;
                    statet->values[1] = fromt->values[1] + diff[1] * t;
                    statet->values[2] = getInterpolatedSO2(fromt->values[2], tot->values[2], t);
                }
                
                for (size_t i = 3; i < moma_param.dof_num + 3; i++)
                    statet->values[i] = fromt->values[i] + diff[i] * t;

                return;
            }

            // void interpolate(const ob::State *from, const ob::State *to, const double t,
            //          ob::State *state) const override
            // {
            //     const auto *fromt = from->as<StateType>();
            //     const auto *tot = to->as<StateType>();
            //     auto *statet = state->as<StateType>();

            //     Eigen::VectorXd diff(moma_param.dof_num+3);
            //     for (size_t i = 0; i < moma_param.dof_num + 3; i++)
            //         diff(i) = (tot->values[i] - fromt->values[i]);

            //     double theta_direct = atan2(diff[1], diff[0]);
            //     double theta_direct_inv = atan2(-diff[1], -diff[0]);
            //     double dist_so2_start = distSO2(fromt->values[2], theta_direct);
            //     double dist_so2_end = distSO2(tot->values[2], theta_direct);
            //     double dist_so2_start_inv = distSO2(fromt->values[2], theta_direct_inv);
            //     double dist_so2_end_inv = distSO2(tot->values[2], theta_direct_inv);
            //     double dist_so2 = dist_so2_start + dist_so2_end;
            //     double dist_so2_inv = dist_so2_start_inv + dist_so2_end_inv;
            //     double dist_r2 = diff.head(2).norm();

            //     double theta_mid = theta_direct;
            //     double dist_start = dist_so2_start;
            //     double dist_end = dist_so2_end;
            //     double dist_theta = dist_so2;
            //     if (dist_so2_inv < dist_so2)
            //     {
            //         theta_mid = theta_direct_inv;
            //         dist_start = dist_so2_start_inv;
            //         dist_end = dist_so2_end_inv;
            //         dist_theta = dist_so2_inv;
            //     }

            //     double t_total = distance(from, to);
            //     double chassis_time = dist_r2 / moma_param.max_v + dist_theta / moma_param.max_w;
            //     double chassis_v = chassis_time * moma_param.max_v / t_total;
            //     double chassis_w = chassis_time * moma_param.max_w / t_total;

            //     if (dist_start > t * t_total * chassis_w)
            //     {
            //         statet->values[0] = fromt->values[0];
            //         statet->values[1] = fromt->values[1];
            //         statet->values[2] = getInterpolatedSO2(fromt->values[2], theta_mid, t * t_total * chassis_w / dist_start);
            //     }
            //     else if (dist_r2 > (t * t_total - dist_start / chassis_w) * chassis_v)
            //     {
            //         statet->values[0] = fromt->values[0] + diff(0) * (t * t_total - dist_start / chassis_w) * chassis_v / dist_r2;
            //         statet->values[1] = fromt->values[1] + diff(1) * (t * t_total - dist_start / chassis_w) * chassis_v / dist_r2;
            //         statet->values[2] = theta_mid;
            //     }
            //     else
            //     {
            //         double temp = t* t_total - dist_start / chassis_w - dist_r2 / chassis_v;
            //         statet->values[0] = tot->values[0];
            //         statet->values[1] = tot->values[1];
            //         statet->values[2] = getInterpolatedSO2(theta_mid, tot->values[2], temp * chassis_w / dist_end);
            //     }

            //     for (size_t i = 3; i < moma_param.dof_num + 3; i++)
            //     {
            //         double joint_diff = (tot->values[i] - fromt->values[i]);
            //         statet->values[i] = fromt->values[i] + diff[i] * t;
            //     }

            //     return;
            // }
    };

    class MomaMotionValidator : public ob::MotionValidator
    {
        private:
            double check_colli_res = 0.01;

        public:
            MomaMotionValidator(const ob::SpaceInformationPtr& si) : ob::MotionValidator(si) {}
            bool checkMotion(const ob::State *s1, const ob::State *s2) const override
            {
                auto stateSpace_ = si_->getStateSpace();
                double time_consume = stateSpace_->distance(s1, s2);
                double piece_num_temp = 1.0 * std::max((int) (time_consume / check_colli_res), 3);
                ob::State* moma_state = si_->allocState();
                for(int i = 0; i < piece_num_temp; ++i)
                {
                    
                    stateSpace_->interpolate(s1, s2, (double) i / (double) piece_num_temp, moma_state);
                    if (!si_->isValid(moma_state))
                    {
                        si_->freeState(moma_state);
                        return false;
                    }
                }
                si_->freeState(moma_state);

                return true;
            }

            bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override
            {
                /* assume motion starts in a valid configuration so s1 is valid */

                double time_consume = si_->getStateSpace()->distance(s1, s2);
                double piece_num = 1.0 * std::max((int) (time_consume/check_colli_res), 3);
                auto stateSpace_ = si_->getStateSpace();
                bool result = true;
                ob::State* moma_state = si_->allocState();
                for(int i = 1; i < piece_num; ++i)
                {
                    stateSpace_->interpolate(s1, s2, (double) 1 / (double) piece_num, moma_state);
                    if (!si_->isValid(moma_state))               
                    {
                        lastValid.second = (double)(i - 1) / (double)piece_num;
                        if (lastValid.first != nullptr)
                            stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                        result = false;
                        break;
                    }
                }
                si_->freeState(moma_state);

                if (result)
                    if (!si_->isValid(s2))
                    {
                        lastValid.second = (double)(piece_num - 1) / (double)piece_num;
                        if (lastValid.first != nullptr)
                            stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                        result = false;
                    }

                if (result)
                    valid_++;
                else
                    invalid_++;

                return result;
            }
    };

    class MomaSimplifier : public og::PathSimplifier
    {
        public:
            using og::PathSimplifier::PathSimplifier;
            MomaSimplifier(const ob::SpaceInformationPtr si) : og::PathSimplifier(si) {}
            bool simplifyMax(og::PathGeometric& path) {
                std::cout << "[OMPL] Start simplifyMax" << std::endl;
                const ob::SpaceInformationPtr& si = path.getSpaceInformation();
                auto space = std::dynamic_pointer_cast<MomaStateSpace>(si->getStateSpace());
                MomaParam moma_param = space->moma_param;
                
                std::cout << "[OMPL] Reduce Vertice" << std::endl;
                
                PathSimplifier::reduceVertices(path);
                std::vector<ob::State *> &states = path.getStates();

                std::cout << "[OMPL] Simplify Path" << std::endl;
                for (size_t i = 1; i < path.getStateCount() - 1; i++) {
                    const MomaStateSpace::StateType *this_state = path.getState(i)->as<MomaStateSpace::StateType>();
                    const MomaStateSpace::StateType *next_state = path.getState(i+1)->as<MomaStateSpace::StateType>();
                    

                    double direct_theta = atan2(
                        next_state->values[1] - this_state->values[1],
                        next_state->values[0] - this_state->values[0]
                    );
                   
                    double dist_so2 = space->distSO2(this_state->values[2], direct_theta);
                    double time = dist_so2 / moma_param.max_w;
                    double total_time = space->distance(this_state, next_state);

                    ob::State* new_state = si->allocState();
                    // for(size_t i = 0; i < moma_param.dof_num+3; i++)
                    //     new_state->as<MomaStateSpace::StateType>()->values[i] = this_state->values[i];
                    // new_state->as<MomaStateSpace::StateType>()->values[2] = direct_theta;
                    space->interpolate(this_state, next_state, time / total_time, new_state);

                    if (si->checkMotion(new_state, states[i+1]) && si->checkMotion(new_state, states[i-1])){
                        auto old_state = states[i];
                        states.erase(states.begin() + i);
                        si->freeState(old_state);
                        states.insert(states.begin() + i, new_state);
                    }
                }
                
                return true;
            }
    };

    class OMPLPlanner
    {
        private:
            GridMap::Ptr grid_map;
            MomaParam moma_param;
            ob::StateSpacePtr space;
            ob::SpaceInformationPtr si;
            og::PRMstar *prm_planner;
            double plan_time;
            double construct_time;
            double inter_time;
            bool use_inter;
            std::string planner_type;
            
        public:
            OMPLPlanner(GridMap::Ptr grid_map_) : grid_map(grid_map_) {}

            ~OMPLPlanner() {};

            inline void init(ros::NodeHandle& nh)
            {
                space = ob::StateSpacePtr(new MomaStateSpace(grid_map, moma_param));
                si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
                si->setStateValidityChecker(std::bind(&OMPLPlanner::isStateValid, this, std::placeholders::_1));
                si->setMotionValidator(std::make_shared<MomaMotionValidator>(si));
                si->setup();

                // setup prm planner
                std::string filename = ros::package::getPath("planner")+"/roadmap/map";
                std::ifstream file(filename);
                if (file.is_open()) 
                {
                    PRINT_GREEN("[OMPL] Load roadmap from file.");
                    file.close();
                    ob::PlannerData plan_data(si);
                    ob::PlannerDataStorage dataStorage;
                    dataStorage.load(filename.c_str(), plan_data);
                    prm_planner = new og::PRMstar(plan_data);
                }
                else
                {
                    PRINT_YELLOW("[OMPL] New roadmap.");
                    prm_planner = new og::PRMstar(si);
                }

                nh.getParam("ompls/plan_time", plan_time);
                nh.getParam("ompls/construct_time", construct_time);
                nh.getParam("ompls/inter_time", inter_time);
                nh.getParam("ompls/use_inter", use_inter);

                nh.getParam("ompls/planner_type", planner_type);
                
                return;
            }

            inline bool isStateValid(const ob::State *state);//碰撞检测

            bool planRRT(const Eigen::VectorXd& start, 
                        const Eigen::VectorXd& end, 
                        std::vector<Eigen::VectorXd>& path_list);
            void planPRM(const Eigen::VectorXd& start, 
                        const Eigen::VectorXd& end, 
                        std::vector<Eigen::VectorXd>& path_list);
            
            void samplePRM(int traj_num, std::vector<std::vector<Eigen::VectorXd>>& paths_list);
            
            void reduceVertices(std::vector<Eigen::VectorXd>& path_list);

            typedef shared_ptr<OMPLPlanner> Ptr;
    };

    inline bool OMPLPlanner::isStateValid(const ob::State *state)
    {
        const MomaStateSpace::StateType *moma_state = state->as<MomaStateSpace::StateType>();
        Eigen::VectorXd gstate(moma_param.dof_num + 3);
        for (size_t i = 0; i < moma_param.dof_num + 3; i++)
            gstate(i) = moma_state->values[i];
        return !(grid_map->isWholeBodyCollision(gstate));
    }
}