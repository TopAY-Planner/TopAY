#pragma once

#include <thread>
#include <numeric>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "utils/minco.hpp"
#include "utils/lbfgs.hpp"
#include "map/grid_map.h"
#include "fake_moma/moma_param.h"

#include <boost/thread.hpp>

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RowMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

namespace nmoma_planner
{    
    struct MomaTraj
    {
        // params
        double seq_res = 0.1;
        int approx_res = 4;
        
        // data
        bool is_init = false;
        Eigen::Vector3d start_state;
        PolyTrajectory<9, 5> poly_traj;     // theta, arc, mani
        std::vector<Eigen::Vector4d> car_seq;

        MomaTraj() {}
        MomaTraj(PolyTrajectory<9, 5> ploy_traj_, 
                 const Eigen::Vector3d& start_state_) : start_state(start_state_), poly_traj(ploy_traj_)
        {
            double Integral_appr_res = seq_res / approx_res;
            double half_Integral_appr_res = Integral_appr_res / 2.0;
            double Integral_appr_res_1_6 = Integral_appr_res / 6.0;

            Eigen::Vector3d current_state = start_state_;
            car_seq.emplace_back(current_state.x(), current_state.y(), current_state.z(), 0.0);

            int sequence_num = floor(poly_traj.getTotalDuration() / Integral_appr_res);

            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p3 = poly_traj.getPos(0.0).head(2);
            v3 = poly_traj.getVel(0.0).head(2);
            for(int i=0; i<sequence_num; i++)
            {
                p1 = p3; v1 = v3;
                p2 = poly_traj.getPos(i * Integral_appr_res + half_Integral_appr_res).head(2);
                v2 = poly_traj.getVel(i * Integral_appr_res + half_Integral_appr_res).head(2);
                p3 = poly_traj.getPos(i * Integral_appr_res + Integral_appr_res).head(2);
                v3 = poly_traj.getVel(i * Integral_appr_res + Integral_appr_res).head(2);

                current_state.x() += Integral_appr_res_1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
                current_state.y() += Integral_appr_res_1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
                if(i%approx_res == approx_res - 1)
                    car_seq.emplace_back(current_state.x(), current_state.y(), p3.x(), (i+1) * Integral_appr_res);
            }
            is_init = true;
        }

        void init()
        {
            double Integral_appr_res = seq_res / approx_res;
            double half_Integral_appr_res = Integral_appr_res / 2.0;
            double Integral_appr_res_1_6 = Integral_appr_res / 6.0;

            Eigen::Vector3d current_state = start_state;
            car_seq.emplace_back(current_state.x(), current_state.y(), current_state.z(), 0.0);

            int sequence_num = floor(poly_traj.getTotalDuration() / Integral_appr_res);

            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p3 = poly_traj.getPos(0.0).head(2);
            v3 = poly_traj.getVel(0.0).head(2);
            for(int i=0; i<sequence_num; i++)
            {
                p1 = p3; v1 = v3;
                p2 = poly_traj.getPos(i * Integral_appr_res + half_Integral_appr_res).head(2);
                v2 = poly_traj.getVel(i * Integral_appr_res + half_Integral_appr_res).head(2);
                p3 = poly_traj.getPos(i * Integral_appr_res + Integral_appr_res).head(2);
                v3 = poly_traj.getVel(i * Integral_appr_res + Integral_appr_res).head(2);

                current_state.x() += Integral_appr_res_1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
                current_state.y() += Integral_appr_res_1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
                if(i%approx_res == approx_res - 1)
                    car_seq.emplace_back(current_state.x(), current_state.y(), p3.x(), (i+1) * Integral_appr_res);
            }
            is_init = true;
        }

        void setTraj(const Eigen::VectorXd start,
                    const Eigen::VectorXd durations,
                    const RowMatrixXd coeffsM)
        {
            start_state = start.head(3);
            std::vector<double> durs;
            std::vector<CoefficientMat<9, 5>> coeffs;
            for (int i=0; i<durations.size(); i++)
            {
                durs.push_back(durations(i));
                coeffs.push_back(coeffsM.block(i*9, 0, 9, 6));
            }
            poly_traj = PolyTrajectory<9, 5>(durs, coeffs);
            init();
            return;
        }

        double getTotalDuration() const
        {
            return poly_traj.getTotalDuration();
        }

        Eigen::VectorXd getState(double t) const
        {
            t = std::min(std::max(t, 0.0), getTotalDuration());
            int index = floor(t / seq_res);
            double floor_t = index * seq_res;
            double diff_t = t - floor_t;
            Eigen::Vector4d car_state = car_seq[index];
            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p1 = poly_traj.getPos(floor_t).head(2);
            v1 = poly_traj.getVel(floor_t).head(2);
            p2 = poly_traj.getPos(floor_t + diff_t/2.0).head(2);
            v2 = poly_traj.getVel(floor_t + diff_t/2.0).head(2);
            p3 = poly_traj.getPos(t).head(2);
            v3 = poly_traj.getVel(t).head(2);

            car_state.x() += diff_t/6.0 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            car_state.y() += diff_t/6.0 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
            car_state.z() = p3.x();

            Eigen::VectorXd state;
            state.resize(10);
            state.head(3) = car_state.head(3);
            state.tail(7) = poly_traj.getPos(t).tail(7);

            return state;
        }

        Eigen::VectorXd getDState(double t) const
        {
            Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
            t = std::min(std::max(t, 0.0), getTotalDuration());
            state(0) = poly_traj.getVel(t)(1);
            state(1) = poly_traj.getVel(t)(0);
            state.tail(7) = poly_traj.getVel(t).tail(7);

            return state;
        }

        double normYaw(const double& yaw) const
        {
            double y = yaw;
            while (y > M_PI)
                y-=2*M_PI;
            while (y < -M_PI)
                y+=2*M_PI;
            return y;
        }
        
        RowMatrixXd sampleTimePoints(int n) const
        {
            int num = n-2;
            double dt = getTotalDuration() / (num+1);
            
            std::vector<Eigen::VectorXd> points_vec;
            for (int i = 0; i<=num; i++)
            {
                Eigen::VectorXd x = getState(i*dt);
                x(2) = normYaw(x(2));
                points_vec.push_back(x);
            }
            Eigen::VectorXd x = getState(getTotalDuration());
            x(2) = normYaw(x(2));
            points_vec.push_back(x);
            assert((int) points_vec.size() == num+2);
            
            RowMatrixXd points;
            points.resize(points_vec.size(), 12);
            for (size_t i = 0; i < points_vec.size(); i++)
            {
                points.row(i).head(10) = points_vec[i];
                points.row(i).tail(2) = Eigen::Vector2d(cos(points_vec[i](2)), sin(points_vec[i](2)));
            }
            return points;
        }

        RowMatrixXd sampleArcPoints(int n) const
        {
            int num = n-2;

            // get ds
            double length = 0.0;
            double dt = 0.1;
            Eigen::VectorXd xlast = getState(0.0);
            for (double t = dt; t <= getTotalDuration(); t += dt)
            {
                Eigen::VectorXd x = getState(t);
                length += (x-xlast).head(2).norm();
                xlast = x;
            }
            double ds = length / (num+1);
            // push points
            RowMatrixXd points;
            std::vector<Eigen::VectorXd> points_vec;
            points_vec.push_back(getState(0.0));
            double temp_len = 0.0;
            xlast = points_vec.back();
            int k = 1;
            for (double t = dt; t < getTotalDuration(); t += dt)
            {
                Eigen::VectorXd x = getState(t);
                temp_len += (x-xlast).head(2).norm();
                if (temp_len > ds*k)
                {
                    points_vec.push_back(x);
                    k++;
                }
                xlast = x;
                if ((int)points_vec.size() == num+1)
                    break;
            }
            points_vec.push_back(getState(getTotalDuration()));

            if ((int)points_vec.size() != num+2)
            {
                points.resize(1, 10);
                return points;
            }

            // assert((int) points_vec.size() == num+2);

            points.resize(points_vec.size(), 10);
            for (size_t i = 0; i < points_vec.size(); i++)
                points.row(i) = points_vec[i].head(10);
            return points;
        }
    };

    struct MobileTraj
    {
        // params
        double seq_res = 0.1;
        int approx_res = 4;
        
        // data
        Eigen::Vector3d start_state;
        PolyTrajectory<2, 5> poly_traj;     // theta, arc
        std::vector<Eigen::Vector4d> car_seq;

        MobileTraj() {}

        MobileTraj(MomaTraj traj)
        {
            Eigen::VectorXd durs_s = traj.poly_traj.getDurations();
            std::vector<double> durs;
            std::vector<CoefficientMat<2, 5>> coeffs;
            for (int pi = 0; pi<traj.poly_traj.getPieceNum(); pi++)
            {
                durs.push_back(durs_s[pi]);
                CoefficientMat<2, 5> now_mat = traj.poly_traj[pi].getCoeffMat().topRows(2);
                coeffs.push_back(now_mat);
            }
            start_state = traj.start_state;
            poly_traj = PolyTrajectory<2, 5>(durs, coeffs);
            init();
            return;
        }

        MobileTraj(PolyTrajectory<2, 5> ploy_traj_, 
                 const Eigen::Vector3d& start_state_) : start_state(start_state_), poly_traj(ploy_traj_)
        {
            double Integral_appr_res = seq_res / approx_res;
            double half_Integral_appr_res = Integral_appr_res / 2.0;
            double Integral_appr_res_1_6 = Integral_appr_res / 6.0;

            Eigen::Vector3d current_state = start_state_;
            car_seq.emplace_back(current_state.x(), current_state.y(), current_state.z(), 0.0);

            int sequence_num = floor(poly_traj.getTotalDuration() / Integral_appr_res);

            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p3 = poly_traj.getPos(0.0).head(2);
            v3 = poly_traj.getVel(0.0).head(2);
            for(int i=0; i<sequence_num; i++)
            {
                p1 = p3; v1 = v3;
                p2 = poly_traj.getPos(i * Integral_appr_res + half_Integral_appr_res).head(2);
                v2 = poly_traj.getVel(i * Integral_appr_res + half_Integral_appr_res).head(2);
                p3 = poly_traj.getPos(i * Integral_appr_res + Integral_appr_res).head(2);
                v3 = poly_traj.getVel(i * Integral_appr_res + Integral_appr_res).head(2);

                current_state.x() += Integral_appr_res_1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
                current_state.y() += Integral_appr_res_1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
                if(i%approx_res == approx_res - 1)
                    car_seq.emplace_back(current_state.x(), current_state.y(), p3.x(), (i+1) * Integral_appr_res);
            }
        }

        void init()
        {
            double Integral_appr_res = seq_res / approx_res;
            double half_Integral_appr_res = Integral_appr_res / 2.0;
            double Integral_appr_res_1_6 = Integral_appr_res / 6.0;

            Eigen::Vector3d current_state = start_state;
            car_seq.emplace_back(current_state.x(), current_state.y(), current_state.z(), 0.0);

            int sequence_num = floor(poly_traj.getTotalDuration() / Integral_appr_res);

            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p3 = poly_traj.getPos(0.0).head(2);
            v3 = poly_traj.getVel(0.0).head(2);
            for(int i=0; i<sequence_num; i++)
            {
                p1 = p3; v1 = v3;
                p2 = poly_traj.getPos(i * Integral_appr_res + half_Integral_appr_res).head(2);
                v2 = poly_traj.getVel(i * Integral_appr_res + half_Integral_appr_res).head(2);
                p3 = poly_traj.getPos(i * Integral_appr_res + Integral_appr_res).head(2);
                v3 = poly_traj.getVel(i * Integral_appr_res + Integral_appr_res).head(2);

                current_state.x() += Integral_appr_res_1_6 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
                current_state.y() += Integral_appr_res_1_6 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
                if(i%approx_res == approx_res - 1)
                    car_seq.emplace_back(current_state.x(), current_state.y(), p3.x(), (i+1) * Integral_appr_res);
            }
        }

        void setTraj(const Eigen::VectorXd start,
                    const Eigen::VectorXd durations,
                    const RowMatrixXd coeffsM)
        {
            start_state = start.head(3);
            std::vector<double> durs;
            std::vector<CoefficientMat<2, 5>> coeffs;
            for (int i=0; i<durations.size(); i++)
            {
                durs.push_back(durations(i));
                coeffs.push_back(coeffsM.block(i*2, 0, 2, 6));
            }
            poly_traj = PolyTrajectory<2, 5>(durs, coeffs);
            init();
            return;
        }

        double getTotalDuration() const
        {
            return poly_traj.getTotalDuration();
        }

        Eigen::Vector3d getState(double t) const
        {
            t = std::min(std::max(t, 0.0), getTotalDuration());
            int index = floor(t / seq_res);
            double floor_t = index * seq_res;
            double diff_t = t - floor_t;
            Eigen::Vector4d car_state = car_seq[index];
            Eigen::Vector2d p1, p2, p3, v1, v2, v3;
            p1 = poly_traj.getPos(floor_t).head(2);
            v1 = poly_traj.getVel(floor_t).head(2);
            p2 = poly_traj.getPos(floor_t + diff_t/2.0).head(2);
            v2 = poly_traj.getVel(floor_t + diff_t/2.0).head(2);
            p3 = poly_traj.getPos(t).head(2);
            v3 = poly_traj.getVel(t).head(2);

            car_state.x() += diff_t/6.0 * (v1.y()*cos(p1.x()) + 4.0*v2.y()*cos(p2.x()) + v3.y()*cos(p3.x()));
            car_state.y() += diff_t/6.0 * (v1.y()*sin(p1.x()) + 4.0*v2.y()*sin(p2.x()) + v3.y()*sin(p3.x()));
            car_state.z() = p3.x();

            return car_state.head(3);
        }

        RowMatrixXd sampleArcPoints(int n) const
        {
            int num = n-2;

            // get ds
            double length = 0.0;
            double dt = 0.05;
            Eigen::VectorXd xlast = getState(0.0);
            for (double t = dt; t <= getTotalDuration(); t += dt)
            {
                Eigen::VectorXd x = getState(t);
                length += (x-xlast).head(2).norm();
                xlast = x;
            }
            double ds = length / (num+1);
            // push points
            RowMatrixXd points;
            std::vector<Eigen::VectorXd> points_vec;
            points_vec.push_back(getState(0.0));
            double temp_len = 0.0;
            xlast = points_vec.back();
            int k = 1;
            for (double t = dt; t < getTotalDuration(); t += dt)
            {
                Eigen::VectorXd x = getState(t);
                temp_len += (x-xlast).head(2).norm();
                if (temp_len > ds*k)
                {
                    points_vec.push_back(x);
                    k++;
                }
                xlast = x;
                if ((int)points_vec.size() == num+1)
                    break;
            }
            points_vec.push_back(getState(getTotalDuration()));

            if (points_vec.size() != num+2)
            {
                points = RowMatrixXd::Ones(num+2, 3);
                return points;
            }

            points.resize(points_vec.size(), 3);
            for (size_t i = 0; i < points_vec.size(); i++)
                points.row(i) = points_vec[i].head(3);
            return points;
        }
    };

    struct ALMParam
    {
        Eigen::VectorXd init_lambda;
        Eigen::VectorXd init_rho;
        Eigen::VectorXd rho_max;
        Eigen::VectorXd gamma;
        Eigen::VectorXd tolerance;
    };

    struct FirstStageParam
    {
        double time_weight;
        double moment_weight;
        double acc_weight;
        double domega_weight;
        double mean_time_weight;
        double path_pos_weight;

        // lbfgs params
        int lbgfs_normal_past;
        int lbgfs_shot_path_past;
        double shot_path_horizon;
        lbfgs::lbfgs_parameter_t lbfgs_param;
    };

    struct FullALMData
    {
        // alm data
        double rho = 1.0;
        double rho_init = 1.0;
        double beta = 1000.0;
        double gamma = 1.0;
        double epsilon_con;
        int max_iter;

        int equal_num;
        int non_equal_num;
        Eigen::VectorXd lambda;
        Eigen::VectorXd mu;
        Eigen::VectorXd hx;
        Eigen::VectorXd gx;
        double scale_fx;
        Eigen::VectorXd scale_cx;
        
        // reset
        inline void reset(int eq_num, int non_eq_num)
        {
            equal_num = eq_num;
            non_equal_num = non_eq_num;
            hx.resize(equal_num);
            hx.setZero();
            lambda.resize(equal_num);
            lambda.setZero();
            gx.resize(non_equal_num);
            gx.setZero();
            mu.resize(non_equal_num);
            mu.setZero();
            scale_fx = 1.0;
            scale_cx.resize(equal_num+non_equal_num);
            scale_cx.setConstant(1.0);
            rho = rho_init;
            return;
        }

        // update dual varables
        inline void updateDualVars()
        {
            lambda += rho * hx;
            for(int i = 0; i < non_equal_num; i++)
                mu(i) = std::max(mu(i)+rho*gx(i), 0.0);
            rho = std::min((1 + gamma) * rho, beta);
        }

        // convergence judgement
        inline bool judgeConvergence()
        {
            if (std::max(hx.lpNorm<Eigen::Infinity>(), \
                        gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>()) < epsilon_con)
            {
                return true;
            }

            return false;
        }
    };

    struct SecondStageParam
    {
        double time_weight;
        double moment_weight;
        double acc_weight;
        double domega_weight;
        double collision_weight;
        double mani_colli_weight;
        double self_colli_weight;
        double mani_pos_weight;
        double mani_vel_weight;
        double mani_acc_weight;
        double mean_time_weight;
        ALMParam alm_param;
        FullALMData alm_data;
        
        lbfgs::lbfgs_parameter_t lbfgs_param;

        void printInfo()
        {
            PRINT_GREEN("Second stage param:");
            PRINT_GREEN("time_weight: " << time_weight);
            PRINT_GREEN("moment_weight: " << moment_weight);
            PRINT_GREEN("acc_weight: " << acc_weight);
            PRINT_GREEN("domega_weight: " << domega_weight);
            PRINT_GREEN("collision_weight: " << collision_weight);
            PRINT_GREEN("mani_colli_weight: " << mani_colli_weight);
            PRINT_GREEN("self_colli_weight: " << self_colli_weight);
            PRINT_GREEN("mani_pos_weight: " << mani_pos_weight);
            PRINT_GREEN("mani_vel_weight: " << mani_vel_weight);
            PRINT_GREEN("mani_acc_weight: " << mani_acc_weight);
            PRINT_GREEN("mean_time_weight: " << mean_time_weight);
        }
    };

    struct MomaTrajOptParam
    {
        int int_K;
        int min_piece_num;
        double relu_mu;
        double sample_interval;
        double mean_time_lowb;
        double mean_time_uppb;
        Eigen::VectorXd energy_weights;
        FirstStageParam first_stage;
        SecondStageParam second_stage;
    };

    class DebugManager
    {
        public:
            std::unordered_map<std::string, std::pair<std_msgs::Float64, ros::Publisher>> datas;
            DebugManager() {}
            void reset()
            {
                for (auto& l:datas)
                    l.second.first.data = 0.0;
            }
            void init(const std::vector<std::string>& cost_names, ros::NodeHandle& nh)
            {
                std_msgs::Float64 d;
                d.data = 0.0;
                for (size_t i=0; i<cost_names.size(); i++)
                    datas.insert(std::make_pair(cost_names[i], std::make_pair(d, nh.advertise<std_msgs::Float64>("/debug_cost/"+cost_names[i], 1))));
            }
            void publish()
            {
                for (auto& l:datas)
                    l.second.second.publish(l.second.first);
            }
            void printinfo()
            {
                for (auto& l:datas)
                    PRINT_YELLOW(l.first << " : " << l.second.first.data);
            }
            void setValue(std::string n, double v)
            {
                if (!(datas.find(n) == datas.end()))
                    datas[n].first.data = v;
                else
                    std::cout<<"ERROR! Can't find cost named "<<n<<"!!!"<<std::endl;
            }
            double& operator[](const std::string& n)
            {
                return datas[n].first.data;
            }
            bool checkInf()
            {
                for (auto& l:datas)
                    if (std::isinf(l.second.first.data) || std::isnan(l.second.first.data))
                        return true;
                return false;
            }
    };

    class MomaTrajOpt
    {
        public:
            MomaTrajOptParam opt_param;
            double traj_cost;
            MomaTraj init_traj;
            MomaTraj afirst_traj;

        private:
            // params
            MomaParam moma_param;

            //data
            int piece_num;
            Eigen::VectorXd start_state;
            Eigen::VectorXd end_state;
            Eigen::VectorXd times;
            Eigen::MatrixXd minco_start_state;  // theta, arc, mani
            Eigen::MatrixXd minco_end_state;    // theta, arc, mani
            Eigen::MatrixXd inner_pts;          // theta, arc, mani
            std::vector<Eigen::Vector2d> init_inner_xy;
            GridMap::Ptr grid_map;
            MinJerkOpt<9> minco_opt;
            Eigen::VectorXd alm_lambda, alm_rho;
            Eigen::Vector2d final_xy_error;
            Eigen::VectorXd ee_pose;
            Eigen::VectorXd final_pose_error;

            // ros
            ros::Publisher debug_whole_pub;
            ros::Publisher debug_car_pub;
            bool debugm = false;

        public:
            typedef std::shared_ptr<MomaTrajOpt> Ptr;
            typedef std::unique_ptr<MomaTrajOpt> UniPtr;
            DebugManager debug_manager;

            MomaTrajOpt(GridMap::Ptr grid_map_) : grid_map(grid_map_) {}
            inline void init(ros::NodeHandle& nh);
            inline MomaTraj getTraj() const;
            inline bool checkFeasible(MomaTraj traj);
            inline bool printConstraintsSituations(MomaTraj traj);
            inline void pubDebugTraj(const MomaTraj& traj);

            bool optimizeEE(Eigen::VectorXd& moma_pos, const Eigen::VectorXd& ee_ref);
            static double eeCostCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);

            bool optimizeTraj(std::vector<Eigen::VectorXd> init_path, 
                              const Eigen::MatrixXd& boundary_vel_, 
                              const Eigen::MatrixXd& boundary_acc_);
            
            bool optimizeTrajNN(std::vector<Eigen::VectorXd> init_path, 
                                const Eigen::MatrixXd& boundary_vel_, 
                                const Eigen::MatrixXd& boundary_acc_);

            static double firstStageCostCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
            static double secondStageCostCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
            static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                                const double fx, const double step, int k, int ls);
            void calFirstStagePenalGrad(double& cost, Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT);
            void calSecondStagePenalGrad(double& cost, Eigen::MatrixXd& gdC, Eigen::VectorXd &gdT);

            double getDurationTrapezoid(const double &length, 
                                        const double &startV, const double &endV, 
                                        const double &maxV, const double &maxA)
            {
                double critical_len; 
                double startv2 = startV * startV;
                double endv2 = endV * endV;
                double maxv2 = maxV * maxV;
                if(startV>maxV)
                    startv2 = maxv2;
                if(endV>maxV)
                    endv2 = maxv2;
                critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
                if(length>=critical_len)
                    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
                else
                {
                    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));
                    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
                }
                return 0.0;
            }

            double getArcTrapezoid(const double &curt, const double &locallength, 
                                   const double &startV, const double &endV, 
                                   const double &maxV, const double &maxA)
            {
                double critical_len; 
                double startv2 = startV * startV;
                double endv2 = endV * endV;
                double maxv2 = maxV * maxV;
                if(startV>maxV)
                    startv2 = maxv2;
                if(endV>maxV)
                    endv2 = maxv2;
                critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
                if(locallength>=critical_len)
                {
                    double t1 = (maxV-startV)/maxA;
                    double t2 = t1+(locallength-critical_len)/maxV;
                    if(curt<=t1)
                        return startV*curt + 0.5*maxA*(curt*curt);
                    else if(curt<=t2)
                        return startV*t1 + 0.5*maxA*(t1*t1)+(curt-t1)*maxV;
                    else
                        return startV*t1 + 0.5*maxA*(t1*t1) + (t2-t1)*maxV + maxV*(curt-t2)-0.5*maxA*(curt-t2)*(curt-t2);
                }
                else
                {
                    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
                    double tmpt = (tmpv-startV)/maxA;
                    if(curt<=tmpt)
                        return startV*curt+0.5*maxA*(curt*curt);
                    else
                        return startV*tmpt+0.5*maxA*(tmpt*tmpt) + tmpv*(curt-tmpt)-0.5*maxA*(curt-tmpt)*(curt-tmpt);
                }
                return 0.0;
            }

            void static normalizeAngle(const double &ref_angle, double &angle)
            {
                while(ref_angle - angle > M_PI)
                    angle += 2*M_PI;
                while(ref_angle - angle < -M_PI)
                    angle -= 2*M_PI;
                return;
            }

            // T = e^τ
            inline double expC2(const double& tau)
            {
                return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
            }

            // τ = ln(T)
            inline double logC2(const double& T)
            {
                return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
            }

            // get dT/dτ
            inline double getTtoTauGrad(const double& tau)
            {
                if (tau > 0)
                    return tau + 1.0;
                else 
                {
                    double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
                    return (1.0 - tau) / (denSqrt * denSqrt);
                } 
            }

            // know τ
            // then get T (uniform)
            inline void calTfromTauUni(const double& tau, Eigen::VectorXd& T)
            {
                T.setConstant(expC2(tau) / T.size());
                return;
            }

            // know τ
            // then get T
            inline void calTfromTau(const Eigen::VectorXd& tau, Eigen::VectorXd& T)
            {
                T.resize(tau.size());
                for (int i=0; i<tau.size(); i++)
                {
                    T(i) = expC2(tau(i));
                }
                return;
            }

            // vq = sigmoid(q)
            inline double sigmoidC2(const double& vq, double max_q)
            {
                double e_ang = expC2(vq);
                return 2.0 * max_q * e_ang / (1.0 + e_ang) - max_q;
            }

            // q = invSigmoid(vq)
            inline double invSigmoidC2(const double& q, double max_q)
            {
                double b = 0.5 * (max_q + q) / max_q;
                return logC2(b/(1-b));
            }

            // get dq/dvq
            inline double getQtoVqGrad(const double& vq, double max_q)
            {
                double e_ang_1 = expC2(vq) + 1.0;
                return 2.0 * max_q * getTtoTauGrad(vq) / (e_ang_1 * e_ang_1);
            }

            // reLu
            inline void smoothL1Penalty(const double& x, double& f, double &df)
            {
                const double pe = opt_param.relu_mu;
                const double half = 0.5 * pe;
                const double f3c = 1.0 / (pe * pe);
                const double f4c = -0.5 * f3c / pe;
                const double d2c = 3.0 * f3c;
                const double d3c = 4.0 * f4c;

                if (x < pe)
                {
                    f = (f4c * x + f3c) * x * x * x;
                    df = (d3c * x + d2c) * x * x;
                }
                else
                {
                    f = x - half;
                    df = 1.0;
                }
                return;
            }

            // get 0.5ρ(h + λ/ρ)^2 or 0.5ρ(g + μ/ρ)^2
            inline double getAugmentedCost(double rho, double h_or_g, double lambda_or_mu)
            {
                return h_or_g * (lambda_or_mu + 0.5*rho*h_or_g);
            }
            
            // get ρh+λ or ρg+μ, the gradient of `getAugmentedCost`
            inline double getAugmentedGrad(double rho, double h_or_g, double lambda_or_mu)
            {
                return rho * h_or_g + lambda_or_mu;
            }
    };

    inline void MomaTrajOpt::init(ros::NodeHandle& nh)
    {
        nh.getParam("moma_traj_opt/int_K", opt_param.int_K);
        nh.getParam("moma_traj_opt/min_piece_num", opt_param.min_piece_num);
        nh.getParam("moma_traj_opt/relu_mu", opt_param.relu_mu);
        nh.getParam("moma_traj_opt/mean_time_lowb", opt_param.mean_time_lowb);
        nh.getParam("moma_traj_opt/mean_time_uppb", opt_param.mean_time_uppb);
        nh.getParam("moma_traj_opt/sample_interval", opt_param.sample_interval);
        std::vector<double> tmp_vec;
        nh.getParam("moma_traj_opt/energy_weights", tmp_vec);
        opt_param.energy_weights.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.energy_weights[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/first_stage/time_weight", opt_param.first_stage.time_weight);
        nh.getParam("moma_traj_opt/first_stage/moment_weight", opt_param.first_stage.moment_weight);
        nh.getParam("moma_traj_opt/first_stage/acc_weight", opt_param.first_stage.acc_weight);
        nh.getParam("moma_traj_opt/first_stage/domega_weight", opt_param.first_stage.domega_weight);
        nh.getParam("moma_traj_opt/first_stage/mean_time_weight", opt_param.first_stage.mean_time_weight);
        nh.getParam("moma_traj_opt/first_stage/path_pos_weight", opt_param.first_stage.path_pos_weight);
        nh.getParam("moma_traj_opt/first_stage/lbgfs_normal_past", opt_param.first_stage.lbgfs_normal_past);
        opt_param.first_stage.lbfgs_param.past = opt_param.first_stage.lbgfs_normal_past;
        nh.getParam("moma_traj_opt/first_stage/lbgfs_shot_path_past", opt_param.first_stage.lbgfs_shot_path_past);
        nh.getParam("moma_traj_opt/first_stage/shot_path_horizon", opt_param.first_stage.shot_path_horizon);
        nh.getParam("moma_traj_opt/first_stage/lbfgs/mem_size", opt_param.first_stage.lbfgs_param.mem_size);
        nh.getParam("moma_traj_opt/first_stage/lbfgs/g_epsilon", opt_param.first_stage.lbfgs_param.g_epsilon);
        nh.getParam("moma_traj_opt/first_stage/lbfgs/min_step", opt_param.first_stage.lbfgs_param.min_step);
        nh.getParam("moma_traj_opt/first_stage/lbfgs/delta", opt_param.first_stage.lbfgs_param.delta);
        nh.getParam("moma_traj_opt/first_stage/lbfgs/max_iterations", opt_param.first_stage.lbfgs_param.max_iterations);

        nh.getParam("moma_traj_opt/second_stage/time_weight", opt_param.second_stage.time_weight);
        nh.getParam("moma_traj_opt/second_stage/moment_weight", opt_param.second_stage.moment_weight);
        nh.getParam("moma_traj_opt/second_stage/acc_weight", opt_param.second_stage.acc_weight);
        nh.getParam("moma_traj_opt/second_stage/domega_weight", opt_param.second_stage.domega_weight);
        nh.getParam("moma_traj_opt/second_stage/collision_weight", opt_param.second_stage.collision_weight);
        nh.getParam("moma_traj_opt/second_stage/mani_colli_weight", opt_param.second_stage.mani_colli_weight);
        nh.getParam("moma_traj_opt/second_stage/self_colli_weight", opt_param.second_stage.self_colli_weight);
        nh.getParam("moma_traj_opt/second_stage/mani_pos_weight", opt_param.second_stage.mani_pos_weight);
        nh.getParam("moma_traj_opt/second_stage/mani_vel_weight", opt_param.second_stage.mani_vel_weight);
        nh.getParam("moma_traj_opt/second_stage/mani_acc_weight", opt_param.second_stage.mani_acc_weight);
        nh.getParam("moma_traj_opt/second_stage/mean_time_weight", opt_param.second_stage.mean_time_weight);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/mem_size", opt_param.second_stage.lbfgs_param.mem_size);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/past", opt_param.second_stage.lbfgs_param.past);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/g_epsilon", opt_param.second_stage.lbfgs_param.g_epsilon);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/min_step", opt_param.second_stage.lbfgs_param.min_step);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/delta", opt_param.second_stage.lbfgs_param.delta);
        nh.getParam("moma_traj_opt/second_stage/lbfgs/max_iterations", opt_param.second_stage.lbfgs_param.max_iterations);
        // opt_param.second_stage.printInfo();

        nh.getParam("moma_traj_opt/second_stage/alm_param/init_lambda", tmp_vec);
        opt_param.second_stage.alm_param.init_lambda.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.second_stage.alm_param.init_lambda[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/second_stage/alm_param/init_rho", tmp_vec);
        opt_param.second_stage.alm_param.init_rho.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.second_stage.alm_param.init_rho[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/second_stage/alm_param/rho_max", tmp_vec);
        opt_param.second_stage.alm_param.rho_max.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.second_stage.alm_param.rho_max[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/second_stage/alm_param/gamma", tmp_vec);
        opt_param.second_stage.alm_param.gamma.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.second_stage.alm_param.gamma[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/second_stage/alm_param/tolerance", tmp_vec);
        opt_param.second_stage.alm_param.tolerance.resize(tmp_vec.size());
        for (size_t i=0; i<tmp_vec.size(); i++)
            opt_param.second_stage.alm_param.tolerance[i] = tmp_vec[i];

        nh.getParam("moma_traj_opt/second_stage/alm_data/max_iter", opt_param.second_stage.alm_data.max_iter);
        nh.getParam("moma_traj_opt/second_stage/alm_data/epsilon_con", opt_param.second_stage.alm_data.epsilon_con);
        opt_param.second_stage.alm_data.rho_init = opt_param.second_stage.alm_param.rho_max[0];

        debug_whole_pub = nh.advertise<visualization_msgs::MarkerArray>("/traj_opt/debug_whole_path", 1);
        debug_car_pub = nh.advertise<nav_msgs::Path>("/traj_opt/debug_car_path", 1);

        debug_manager.init({"jerk", 
                            "time", 
                            "chassis_colli", 
                            "moment", 
                            "acc", 
                            "domega", 
                            "mani_colli", 
                            "self_colli", 
                            "mani_pos", 
                            "mani_vel", 
                            "mani_acc",
                            "mean_time", 
                            "endp"}, nh);
        
        return;
    }

    inline MomaTraj MomaTrajOpt::getTraj() const
    {
        return MomaTraj(minco_opt.getTraj(), start_state.head(3));
    }

    inline bool MomaTrajOpt::checkFeasible(MomaTraj traj)
    {
        bool feasible = true;

        double res = 0.01;
        double max_vel = 0.0;
        double max_acc = 0.0;
        double max_domega = 0.0;
        double max_d2omega = 0.0;
        double min_dist = 1.0e+10;
        Eigen::VectorXd max_q; max_q.resize(moma_param.dof_num); max_q.setZero();
        Eigen::VectorXd max_dq; max_dq.resize(moma_param.dof_num); max_dq.setZero();
        Eigen::VectorXd max_d2q; max_d2q.resize(moma_param.dof_num); max_d2q.setZero();
        Eigen::VectorXd temp_state; temp_state.resize(moma_param.dof_num+3); temp_state.setZero();
        std::vector<Eigen::Vector4d> min_dist_mani = moma_param.getColliPts(temp_state);
        for (size_t i=0; i<min_dist_mani.size(); i++)
            min_dist_mani[i].x() = 1.0e+10;

        for (double t=0.0; t<traj.getTotalDuration(); t+=res)
        {
            Eigen::VectorXd state = traj.getState(t);
            Eigen::VectorXd vel = traj.poly_traj.getVel(t);
            Eigen::VectorXd acc = traj.poly_traj.getAcc(t);

            if (fabs(vel(1)) > fabs(max_vel))
                max_vel = vel(1);
            if (fabs(acc(1)) > fabs(max_acc))
                max_acc = acc(1);
            if (fabs(vel(0)) > fabs(max_domega))
                max_domega = vel(0);
            if (fabs(acc(0)) > fabs(max_d2omega))
                max_d2omega = acc(0);
            
            for (size_t i=0; i<moma_param.dof_num; i++)
            {
                if (fabs(state(i+3)) > fabs(max_q(i)))
                    max_q(i) = state(i+3);
                if (fabs(vel(i+2)) > fabs(max_dq(i)))
                    max_dq(i) = vel(i+2);
                if (fabs(acc(i+2)) > fabs(max_d2q(i)))
                    max_d2q(i) = acc(i+2);
            }
            
            double d = 0.0;
            grid_map->getDistance2d(state.head(2), d);
            if (d < min_dist)
                min_dist = d;

            std::vector<Eigen::Vector4d> mani_pts = moma_param.getColliPts(state);
            for (size_t i=0; i<mani_pts.size(); i++)
            {
                double d = 0.0;
                grid_map->getDistance3d(mani_pts[i].head(3), d);
                if (d < min_dist_mani[i].x())
                    min_dist_mani[i].x() = d;
            }
        }

        if (fabs(max_vel) > 1.01 * moma_param.max_v)
            feasible = false;

        if (fabs(max_acc) > 1.01 * moma_param.max_a)
            feasible = false;

        if (fabs(max_domega) > 1.01 * moma_param.max_w)
            feasible = false;

        if (fabs(max_d2omega) > 1.01 * moma_param.max_dw)
            feasible = false;

        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_q(i)) > 1.01 * moma_param.joint_pos_limit_max(i))
            {
                feasible = false;
                break;
            }

        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_dq(i)) > 1.01 * moma_param.joint_vel_limit(i))
            {
                feasible = false;
                break;
            }

        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_d2q(i)) > 1.01 * moma_param.joint_acc_limit(i))
            {
                feasible = false;
                break;
            }

        if (min_dist < 0.99 * moma_param.chassis_colli_radius)
            feasible = false;
    
        for (size_t i=0; i<min_dist_mani.size(); i++)
            if (min_dist_mani[i].x() < 0.99 * min_dist_mani[i].w())
            {
                feasible = false;
                break;
            }

        return feasible;
    }

    inline bool MomaTrajOpt::printConstraintsSituations(MomaTraj traj)
    {
        bool feasible = true;

        double res = 0.01;
        double max_vel = 0.0;
        double max_acc = 0.0;
        double max_domega = 0.0;
        double max_d2omega = 0.0;
        double min_dist = 1.0e+10;
        Eigen::VectorXd max_q; max_q.resize(moma_param.dof_num); max_q.setZero();
        Eigen::VectorXd max_dq; max_dq.resize(moma_param.dof_num); max_dq.setZero();
        Eigen::VectorXd max_d2q; max_d2q.resize(moma_param.dof_num); max_d2q.setZero();
        Eigen::VectorXd temp_state; temp_state.resize(moma_param.dof_num+3); temp_state.setZero();
        std::vector<Eigen::Vector4d> min_dist_mani = moma_param.getColliPts(temp_state);
        for (size_t i=0; i<min_dist_mani.size(); i++)
            min_dist_mani[i].x() = 1.0e+10;

        for (double t=0.0; t<traj.getTotalDuration(); t+=res)
        {
            Eigen::VectorXd state = traj.getState(t);
            Eigen::VectorXd vel = traj.poly_traj.getVel(t);
            Eigen::VectorXd acc = traj.poly_traj.getAcc(t);

            if (fabs(vel(1)) > fabs(max_vel))
                max_vel = vel(1);
            if (fabs(acc(1)) > fabs(max_acc))
                max_acc = acc(1);
            if (fabs(vel(0)) > fabs(max_domega))
                max_domega = vel(0);
            if (fabs(acc(0)) > fabs(max_d2omega))
                max_d2omega = acc(0);
            
            for (size_t i=0; i<moma_param.dof_num; i++)
            {
                if (fabs(state(i+3)) > fabs(max_q(i)))
                    max_q(i) = state(i+3);
                if (fabs(vel(i+2)) > fabs(max_dq(i)))
                    max_dq(i) = vel(i+2);
                if (fabs(acc(i+2)) > fabs(max_d2q(i)))
                    max_d2q(i) = acc(i+2);
            }
            
            double d = 0.0;
            grid_map->getDistance2d(state.head(2), d);
            if (d < min_dist)
                min_dist = d;

            std::vector<Eigen::Vector4d> mani_pts = moma_param.getColliPts(state);
            for (size_t i=0; i<mani_pts.size(); i++)
            {
                double d = 0.0;
                grid_map->getDistance3d(mani_pts[i].head(3), d);
                if (d < min_dist_mani[i].x())
                    min_dist_mani[i].x() = d;
            }
        }

        PRINTF_WHITE("[Moma Opt] traj max velocity: ");
        if (fabs(max_vel) > 1.01 * moma_param.max_v)
        {
            feasible = false;
            PRINT_RED(max_vel);
        }
        else
            PRINTF_WHITE(max_vel<<"\n");
        
        PRINTF_WHITE("[Moma Opt] traj max acceleration: ");
        if (fabs(max_acc) > 1.01 * moma_param.max_a)
        {
            feasible = false;
            PRINT_RED(max_acc);
        }
        else
            PRINTF_WHITE(max_acc<<"\n");

        PRINTF_WHITE("[Moma Opt] traj max domega: ");
        if (fabs(max_domega) > 1.01 * moma_param.max_w)
        {
            feasible = false;
            PRINT_RED(max_domega);
        }
        else
            PRINTF_WHITE(max_domega<<"\n");

        PRINTF_WHITE("[Moma Opt] traj max d2omega: ");
        if (fabs(max_d2omega) > 1.01 * moma_param.max_dw)
        {
            feasible = false;
            PRINT_RED(max_d2omega);
        }
        else
            PRINTF_WHITE(max_d2omega<<"\n");

        PRINTF_WHITE("[Moma Opt] traj max q: ");
        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_q(i)) > 1.01 * moma_param.joint_pos_limit_max(i))
            {
                feasible = false;
                PRINTF_RED(max_q(i)<<" ");
            }
            else
                PRINTF_WHITE(max_q(i)<<" ");
        PRINTF_WHITE("\n");

        PRINTF_WHITE("[Moma Opt] traj max dq: ");
        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_dq(i)) > 1.01 * moma_param.joint_vel_limit(i))
            {
                feasible = false;
                PRINTF_RED(max_dq(i)<<" ");
            }
            else
                PRINTF_WHITE(max_dq(i)<<" ");
        PRINTF_WHITE("\n");

        PRINTF_WHITE("[Moma Opt] traj max d2q: ");
        for (size_t i=0; i<moma_param.dof_num; i++)
            if (fabs(max_d2q(i)) > 1.01 * moma_param.joint_acc_limit(i))
            {
                feasible = false;
                PRINTF_RED(max_d2q(i)<<" ");
            }
            else
                PRINTF_WHITE(max_d2q(i)<<" ");
        PRINTF_WHITE("\n");

        PRINTF_WHITE("[Moma Opt] traj chassis min distance: ");
        if (min_dist < 0.99 * moma_param.chassis_colli_radius)
        {
            feasible = false;
            PRINT_RED(min_dist);
        }
        else
            PRINTF_WHITE(min_dist<<"\n");
    
        PRINTF_WHITE("[Moma Opt] traj manipulator collision constraints:\n");
        for (size_t i=0; i<min_dist_mani.size(); i++)
            PRINTF_YELLOW(min_dist_mani[i].w()<<" ");
        PRINTF_WHITE("\n");
        PRINTF_WHITE("[Moma Opt] traj manipulator min distance:\n");
        for (size_t i=0; i<min_dist_mani.size(); i++)
            if (min_dist_mani[i].x() < 0.99 * min_dist_mani[i].w())
            {
                // feasible = false;
                PRINTF_RED(min_dist_mani[i].x()<<" ");
            }
            else
                PRINTF_WHITE(min_dist_mani[i].x()<<" ");
        PRINTF_WHITE("\n");

        return feasible;
    }

    inline void MomaTrajOpt::pubDebugTraj(const MomaTraj& traj)
    {
        std::vector<Eigen::VectorXd> end_path;
        nav_msgs::Path car_traj;
        Eigen::VectorXd times = traj.poly_traj.getDurations();
        double nowt = 0.0;
        for (size_t i=0; i<times.size()+1; i++)
        {
            Eigen::VectorXd state = traj.getState(nowt);
            end_path.push_back(state);
            car_traj.header.frame_id = "world";
            car_traj.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped gp;
            gp.pose.position.x = state.x();
            gp.pose.position.y = state.y();
            gp.pose.position.z = 0.0;
            gp.pose.orientation.w = cos(state.z()/2.0);
            gp.pose.orientation.x = 0.0;
            gp.pose.orientation.y = 0.0;
            gp.pose.orientation.z = sin(state.z()/2.0);
            car_traj.poses.push_back(gp);
            if (i < times.size())
                nowt += times[i];
        }
        debug_car_pub.publish(car_traj);

        visualization_msgs::Marker line_strip, arrow, text;
        arrow.header.frame_id = line_strip.header.frame_id = text.header.frame_id = "world";
        arrow.header.stamp = line_strip.header.stamp = text.header.stamp = ros::Time::now();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        arrow.type = visualization_msgs::Marker::ARROW;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.id = 1886666;
        text.action = visualization_msgs::Marker::ADD;
        text.scale.z = 0.1;
        text.color.a = 1.0;
        line_strip.id = 10086;
        arrow.scale.x = 0.03;
        arrow.scale.y = 0.05;
        arrow.color.a = 1.0;
        arrow.color.r = 1.0;
        arrow.color.g = 0.0;
        arrow.color.b = 0.0;
        arrow.pose.orientation.w = 1.0;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.03;
        line_strip.scale.y = 0.03;
        line_strip.scale.z = 0.03;
        line_strip.color.a = 1.0;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;

        visualization_msgs::MarkerArray array_msg;
        visualization_msgs::Marker p;
        p.action = visualization_msgs::Marker::DELETEALL;
        p.id = 0;
        array_msg.markers.push_back(p);
        for (size_t i=0; i<end_path.size(); i++)
        {
            visualization_msgs::MarkerArray node_array = moma_param.getColliCylinderArray(end_path[i]);
            size_t array_size = node_array.markers.size();
            for (size_t j=0; j<array_size; j++)
            {
                node_array.markers[j].id = i*array_size+j;
                node_array.markers[j].color.a = 0.3;
                node_array.markers[j].color.r = 0.0;
                node_array.markers[j].color.g = 0.0;
                node_array.markers[j].color.b = 1.0;
                array_msg.markers.push_back(node_array.markers[j]);
            }
            geometry_msgs::Point pt;
            pt.x = end_path[i].x();
            pt.y = end_path[i].y();
            pt.z = 0.0;
            line_strip.points.push_back(pt);
            geometry_msgs::Point pt_arrow;
            pt_arrow.x = end_path[i].x() + moma_param.chassis_colli_radius*cos(end_path[i].z());
            pt_arrow.y = end_path[i].y() + moma_param.chassis_colli_radius*sin(end_path[i].z());
            arrow.points.clear();
            arrow.points.push_back(pt);
            arrow.points.push_back(pt_arrow);
            arrow.id = line_strip.id + i + 1;
            array_msg.markers.push_back(arrow);
            text.color.r = 0.0;
            arrow.color.b = 0.0;
            text.text = std::to_string(i);
            text.id = text.id + 1;
            text.pose.orientation.w = 1.0;
            text.pose.position = node_array.markers.back().pose.position;
            text.pose.position.z = text.pose.position.z + 0.1;
            array_msg.markers.push_back(text);
        }
        array_msg.markers.push_back(line_strip);
        debug_whole_pub.publish(array_msg);

        return;
    }
}