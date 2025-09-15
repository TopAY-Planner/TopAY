#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string.h>
#include <algorithm>
#include <numeric>

#include <casadi/casadi.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "fake_moma/MomaCmd.h"
#include "fake_moma/MomaState.h"
#include "fake_moma/moma_param.h"
#include "planner/moma_traj_opt.h"

using namespace std;
using namespace Eigen;
using namespace casadi;

namespace nmoma_planner
{
    class MPC
    {
    private:
        int state_dim;
        int control_dim;
        int predict_N = 30;
        int delay_num_chassis = 2;
        double dt = 0.1;
        MomaParam moma_param;

        Function solver;

        map<string, DM> arg, res;

        DM U0;
        DM X0;
        DM lbx;
        DM ubx;
        DM lbg;
        DM ubg;

        bool has_traj = false;
        bool at_goal = false;
        MomaTraj traj;
        Eigen::VectorXd last_cmd;
        std::vector<Eigen::Vector2d> vw_buffer;

        ros::Time begin_time;
        ros::Publisher ref_pub;
        ros::Publisher predict_pub;

    public:
        double ctrl_freq = 100.0;
        inline void init(ros::NodeHandle &nh);
        inline void smoothAngle(const Eigen::VectorXd& source, Eigen::VectorXd& target);
        inline MX dynamicFunc(MX state, MX control);
        inline MX stateTrans(MX state, MX control);
        inline void setTraj(const MomaTraj &traj_);
        inline void drawPredictPath(void);
        inline void drawRefPath(const std::vector<VectorXd>& ref_states);
        inline fake_moma::MomaCmd getCmd(Eigen::VectorXd now_state);

        typedef std::shared_ptr<MPC> Ptr;
    };

    inline void MPC::init(ros::NodeHandle &nh)
    {
        // TODO: load params
        vector<double> Q_ ;
        vector<double> R_;
        vector<double> Rd_;

        nh.getParam("mpc/dt", dt);
        nh.getParam("mpc/predict_steps", predict_N);
        nh.getParam("mpc/ctrl_freq", ctrl_freq);
        nh.getParam("mpc/delay_num_chassis", delay_num_chassis);
        nh.param<std::vector<double>>("mpc/matrix_q", Q_, std::vector<double>());
        nh.param<std::vector<double>>("mpc/matrix_r", R_, std::vector<double>());
        nh.param<std::vector<double>>("mpc/matrix_rd", Rd_, std::vector<double>());

        ref_pub = nh.advertise<visualization_msgs::Marker>("/reference_path", 1);
        predict_pub = nh.advertise<visualization_msgs::Marker>("/predict_path", 1);

        MX Q = diag(DM({Q_[0], Q_[1], Q_[2], Q_[3], Q_[4], Q_[5], Q_[6], Q_[7], Q_[8], Q_[9]}));
        MX R = diag(DM({R_[0], R_[1], R_[2], R_[3], R_[4], R_[5], R_[6], R_[7], R_[8]}));
        MX Rd = diag(DM({Rd_[0], Rd_[1], Rd_[2], Rd_[3], Rd_[4], Rd_[5], Rd_[6], Rd_[7], Rd_[8]}));

        state_dim = 3+moma_param.dof_num;
        control_dim = 2+moma_param.dof_num;
        U0 = DM::zeros(control_dim, predict_N);
        X0 = DM::zeros(state_dim, predict_N+1);
        last_cmd.resize(control_dim); last_cmd.setZero();
        MX P = MX::sym("P", state_dim*(predict_N+1)+control_dim, 1);
        MX P_state = reshape(P(Slice(0, state_dim*(predict_N+1))), state_dim, predict_N+1);

        // define cost function
        Slice all;
        Slice chas_slice(0, 3);
        Slice mani_slice(3, (int)(3+moma_param.dof_num));
        Slice vw_slice(0, 2);
        Slice mani_vel_slice(2, (int)(2+moma_param.dof_num));
        MX X = MX::sym("X", state_dim, predict_N+1);
        MX U = MX::sym("U", control_dim, predict_N);
        MX g = X(all, 0) - P_state(all, 0);
        MX cost = 0;
        for (int i = 0; i < delay_num_chassis; i++)
        {
            g = vertcat(g, X(chas_slice, i+1) - P_state(chas_slice, i+1));
        }
        for (int i = 0; i < predict_N; i++)
        {
            MX delta_x = X(all, i) - P_state(all, i);
            cost = cost + mtimes(delta_x.T(), mtimes(Q, delta_x))
                        + mtimes(U(all, i).T(), mtimes(R, U(all, i)));
            if (i>0)
            {
                MX delta_u = U(all, i) - U(all, i-1);
                cost = cost + mtimes(delta_u.T(), mtimes(Rd, delta_u));
            }
            g = vertcat(g, X(all, i+1) - stateTrans(X(all, i), U(all, i)));
        }

        MX P_control = P(Slice(state_dim*(predict_N+1), state_dim*(predict_N+1)+control_dim));
        g = vertcat(g, U(all, 0) - P_control);
        for (int i=1; i<predict_N; i++)
            g = vertcat(g, U(all, i) - U(all, i-1));

        // set limits
        lbx = DM::zeros(state_dim*(predict_N+1) + control_dim*predict_N, 1);
        ubx = DM::zeros(state_dim*(predict_N+1) + control_dim*predict_N, 1);
        lbg = DM::zeros(state_dim*(predict_N+1) + control_dim*predict_N, 1);
        ubg = DM::zeros(state_dim*(predict_N+1) + control_dim*predict_N, 1);
        for (int i = 0; i < predict_N+1; i++)
        {
            for (int j=0; j<3; j++)
            {
                lbx(state_dim*i+j, 0) = -inf;
                ubx(state_dim*i+j, 0) = inf;
            }
            for (size_t j=0; j<moma_param.dof_num; j++)
            {
                lbx(state_dim*i+j+3, 0) = moma_param.joint_pos_limit_min(j);
                ubx(state_dim*i+j+3, 0) = moma_param.joint_pos_limit_max(j);
            }
            if (i < predict_N)
            {
                lbx(state_dim*(predict_N+1)+control_dim*i, 0) = -moma_param.max_v;
                ubx(state_dim*(predict_N+1)+control_dim*i, 0) = moma_param.max_v;
                lbx(state_dim*(predict_N+1)+control_dim*i+1, 0) = -moma_param.max_w;
                ubx(state_dim*(predict_N+1)+control_dim*i+1, 0) = moma_param.max_w;
                for (size_t j=0; j<moma_param.dof_num; j++)
                {
                    lbx(state_dim*(predict_N+1)+control_dim*i+2+j, 0) = -moma_param.joint_vel_limit(j);
                    ubx(state_dim*(predict_N+1)+control_dim*i+2+j, 0) = moma_param.joint_vel_limit(j);
                    lbg(state_dim*(predict_N+1)+control_dim*i+2+j, 0) = -moma_param.joint_acc_limit(j) * dt;
                    ubg(state_dim*(predict_N+1)+control_dim*i+2+j, 0) = moma_param.joint_acc_limit(j) * dt;
                }
                lbg(state_dim*(predict_N+1)+control_dim*i, 0) = -moma_param.max_a * dt;
                ubg(state_dim*(predict_N+1)+control_dim*i, 0) = moma_param.max_a * dt;
                lbg(state_dim*(predict_N+1)+control_dim*i+1, 0) = -moma_param.max_dw * dt;
                ubg(state_dim*(predict_N+1)+control_dim*i+1, 0) = moma_param.max_dw * dt;
            }
        }
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        
        // set optimization problem
        MX opt_var = vertcat(reshape(X, -1, 1), reshape(U, -1, 1));
        MXDict nlp_prob;
        nlp_prob["f"] = cost;
        nlp_prob["x"] = opt_var;
        nlp_prob["g"] = g;
        nlp_prob["p"] = P;

        // casadi::Dict options;
        // options["print_time"] = false;
        // options["ipopt.print_level"] = 5;
        // options["ipopt.max_iter"] = 1000;
        // options["ipopt.acceptable_tol"] = 1e-8;
        // options["ipopt.acceptable_obj_change_tol"] = 1e-6;

        casadi::Dict options;
        casadi::Dict qp_options;
        options["print_status"] = false;
        options["print_time"] = false;
        options["print_header"] = false;
        options["print_iteration"] = false;
        options["verbose"] = false;
        options["verbose_init"] = false;
        qp_options["printLevel"] = "none";
        qp_options["sparse"] = true;
        qp_options["error_on_fail"] = false;
        options["qpsol_options"] = qp_options;

        solver = nlpsol("solver", "sqpmethod", nlp_prob, options);
        // solver = nlpsol("solver", "ipopt", nlp_prob, options);
        return;
    }

    inline MX MPC::dynamicFunc(MX state, MX control)
    {
        MX Dstate = vertcat(control(0) * cos(state(2)),
                            control(0) * sin(state(2)));
        for (size_t i = 0; i < moma_param.dof_num+1; i++)
            Dstate = vertcat(Dstate, control(i+1));
        return Dstate;
    }

    inline MX MPC::stateTrans(MX state, MX control)
    {
        // MX k1 = dynamicFunc(state, control);
        // MX k2 = dynamicFunc(state + dt / 2 * k1, control);
        // MX k3 = dynamicFunc(state + dt / 2 * k2, control);
        // MX k4 = dynamicFunc(state + dt * k3, control);
        // return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);

        MX k1 = dynamicFunc(state, control);
        return state + dt * k1;
    }

    inline void MPC::setTraj(const MomaTraj &traj_)
    {
        traj = traj_;
        begin_time = ros::Time::now();
        has_traj = true;
        at_goal = false;
        return;
    }

    inline void MPC::smoothAngle(const Eigen::VectorXd& source, Eigen::VectorXd& target)
    {
        for (size_t i = 0; i < moma_param.dof_num+1; i++)
        {
            double dyaw = target(2+i) - source(2+i);
            while (dyaw >= M_PI / 2)
            {
                target(2+i) -= M_PI * 2;
                dyaw = target(2+i) - source(2+i);
            }
            while (dyaw <= -M_PI / 2)
            {
                target(2+i) += M_PI * 2;
                dyaw = target(2+i) - source(2+i);
            }
        }
        return;
    }

    inline void MPC::drawPredictPath()
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 1;
        sphere.color.b = line_strip.color.b = 0;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        Slice all;
        for (int i=0; i<predict_N+1; i++)
        {
            DM pre_state = X0(all, i);
            pt.x = double(pre_state(0, 0));
            pt.y = double(pre_state(1, 0));
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub.publish(line_strip);
    }

    inline void MPC::drawRefPath(const std::vector<VectorXd>& ref_states)
    {
        int id = 0;
        double sc = 0.05;
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0;
        sphere.color.g = line_strip.color.g = 0;
        sphere.color.b = line_strip.color.b = 1;
        sphere.color.a = line_strip.color.a = 1;
        sphere.scale.x = sc;
        sphere.scale.y = sc;
        sphere.scale.z = sc;
        line_strip.scale.x = sc / 2;
        geometry_msgs::Point pt;
        
        for (size_t i=0; i<ref_states.size(); i++)
        {
            pt.x = ref_states[i].x();
            pt.y = ref_states[i].y();
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }

    inline fake_moma::MomaCmd MPC::getCmd(Eigen::VectorXd now_state)
    {
        if (!has_traj)
        {
            fake_moma::MomaCmd cmd_msg;
            cmd_msg.speed = 0.0;
            cmd_msg.angular_velocity = 0.0;
            for (size_t i = 0; i < moma_param.dof_num; i++)
            {
                cmd_msg.dq.data.push_back(0.0);
                cmd_msg.q.data.push_back(now_state(3+i));
            }
            return cmd_msg;
        }

        double t_cur = (ros::Time::now() - begin_time).toSec();
        if (t_cur >= traj.getTotalDuration() + 1.0)
            at_goal = true;
        
        if (at_goal)
        {
            fake_moma::MomaCmd cmd_msg;
            cmd_msg.speed = 0.0;
            cmd_msg.angular_velocity = 0.0;
            for (size_t i = 0; i < moma_param.dof_num; i++)
            {
                cmd_msg.dq.data.push_back(0.0);
                cmd_msg.q.data.push_back(now_state(3+i));
            }
            return cmd_msg;
        }

        // get reference state and control
        DM p = DM::zeros(state_dim*(predict_N+1)+control_dim, 1);
        for (int i = 0; i<state_dim; i++)
            p(i) = now_state(i);
        int j=1;
        Eigen::VectorXd last_state = now_state;
        std::vector<VectorXd> ref_states;
        for (double t = t_cur + dt; j<predict_N+1; t += dt, j++)
        {
            Eigen::VectorXd state = traj.getState(t);
            smoothAngle(last_state, state);
            ref_states.push_back(state);
            last_state = state;
            for (int i = 0; i<state_dim; i++)
                p(state_dim*j+i, 0) = state(i);
        }
        for (int i = 0; i<control_dim; i++)
            p(state_dim*(predict_N+1)+i, 0) = last_cmd(i);
        arg["x0"] = vertcat(reshape(X0, -1, 1), reshape(U0, -1, 1));
        arg["p"] = p;

        res = solver(arg);
        X0 = reshape(res["x"](Slice(0, state_dim*(predict_N+1))), state_dim, predict_N+1);
        U0 = reshape(res["x"](Slice(state_dim*(predict_N+1), state_dim*(predict_N+1)+control_dim*predict_N)), control_dim, predict_N);
        
        drawRefPath(ref_states);
        drawPredictPath();

        // publish
        fake_moma::MomaCmd cmd_msg;
        DM control = U0(Slice(), 0);
        cmd_msg.speed = last_cmd(0) = (double)control(0, 0);
        cmd_msg.angular_velocity = last_cmd(1) = (double)control(1, 0);
        for (size_t i = 0; i < moma_param.dof_num; i++)
        {
            last_cmd(2+i) = (double)control(2+i, 0);
            cmd_msg.dq.data.push_back(last_cmd(2+i));
            cmd_msg.q.data.push_back(now_state(3+i)+last_cmd(2+i)/ctrl_freq);
        }

        return cmd_msg;
    }
}
