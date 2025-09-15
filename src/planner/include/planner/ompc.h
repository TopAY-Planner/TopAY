#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <string.h>
#include <algorithm>
 
#include <OsqpEigen/OsqpEigen.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>

#include "fake_moma/MomaCmd.h"
#include "planner/moma_traj_opt.h"

#define DIFF  1

using namespace std;
using namespace Eigen;
using namespace nmoma_planner;

class OMPCState
{
public:
    double x = 0;
    double y = 0;
    double theta = 0;
};

class OMPCInput
{
public:
    double vx = 0;
    double vy = 0;
    double w = 0;
    double delta = 0;
};

typedef pair<OMPCState, OMPCInput> OMPCNode;

class OMPC
{
private:
    // parameters
    /// algorithm param
    int model_type;
    double du_th = 0.1;
    double dt = 0.2;
    int T = 5;
    int max_iter = 3;
    int delay_num_v;
    int delay_num_w;
    int max_delay_num;
    int delay_num_Asub;
    vector<double> Q = {10, 10, 0.5};
    vector<double> R = {0.01, 0.01};
    vector<double> Rd = {0.01, 1.0};
    /// constraints
    double max_omega = M_PI / 4;
    double max_domega = M_PI / 6;
    double max_comega = M_PI / 6 * 0.2;
    double max_speed = 55.0 / 3.6;
    double min_speed = -55.0 / 3.6;
    double max_cv = 0.2;
    double max_accel = 1.0; 

    // OMPC dataset
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd C;
    OMPCNode xbar[500];
    Eigen::MatrixXd xref;
    Eigen::MatrixXd dref;
    Eigen::MatrixXd output;
    Eigen::MatrixXd last_output;
    std::vector<Eigen::Vector2d> output_buff;

    // control data
    bool has_traj = false;
    bool at_goal = true;
    int control_state = 0; // 0: common, 1: in out
    Eigen::Vector3d direct_pos;
    ros::Time begin_time;
    MomaTraj traj;

    OMPCState now_state;

    // ros interface
	ros::NodeHandle node_;
    ros::Publisher predict_pub, ref_pub, refqdq_pub;

    // OMPC function
    void getLinearModel(const OMPCNode& node);
    void stateTrans(OMPCNode& node);
    void predictMotion(void);
    void predictMotion(OMPCState *b);
    void solveMPCDiff(void);

public:
    double ctrl_freq = 10.0;
    
    bool hasTraj() { return has_traj; }
    fake_moma::MomaCmd getCmd(Eigen::VectorXd now_state);
    void pubCmd(Eigen::VectorXd now_state, ros::Publisher &puber, bool gripper = true);
    void pubCmd(Eigen::VectorXd now_state, ros::Publisher &puber, int ctrl_state = 0);
    
    typedef std::shared_ptr<OMPC> Ptr;

    // utils
    OMPCState xopt[500];

    void setTraj(const MomaTraj &traj_)
    {
        traj = traj_;
        begin_time = ros::Time::now();
        has_traj = true;
        at_goal = false;
        control_state = 0;
    }

    void setTraj(const MomaTraj &traj_, double time_cunsume)
    {
        traj = traj_;
        begin_time = ros::Time::now() - ros::Duration(time_cunsume);
        has_traj = true;
        at_goal = false;
        control_state = 0;
    }

    void setDirect(const Eigen::Vector3d &direct_pos_)
    {
        direct_pos = direct_pos_;
        begin_time = ros::Time::now();
        has_traj = true;
        at_goal = false;
        control_state = 1;
    }

    void normlize_theta(double& th)
    {
        while (th > M_PI)
            th -= M_PI * 2;
        while (th < -M_PI)
            th += M_PI * 2;
    }

    void smooth_yaw(void)
    {
        double dyaw = xref(2, 0) - now_state.theta;

        while (dyaw >= M_PI / 2)
        {
            xref(2, 0) -= M_PI * 2;
            dyaw = xref(2, 0) - now_state.theta;
        }
        while (dyaw <= -M_PI / 2)
        {
            xref(2, 0) += M_PI * 2;
            dyaw = xref(2, 0) - now_state.theta;
        }

        for (int i=0; i<T-1; i++)
        {
            dyaw = xref(2, i+1) - xref(2, i);
            while (dyaw >= M_PI / 2)
            {
                xref(2, i+1) -= M_PI * 2;
                dyaw = xref(2, i+1) - xref(2, i);
            }
            while (dyaw <= -M_PI / 2)
            {
                xref(2, i+1) += M_PI * 2;
                dyaw = xref(2, i+1) - xref(2, i);
            }
        }
    }

    void drawPredictPath(OMPCState *b)
    {
        int id = 0;
        double sc = 0.1;
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
        
        for (int i=0; i<T; i++)
        {
            pt.x = b[i].x;
            pt.y = b[i].y;
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        predict_pub.publish(line_strip);
    }

    void drawRefQdQ(const Eigen::VectorXd& q,
                    const Eigen::VectorXd& dq)
    {
        std_msgs::Float32MultiArray qdq;
        for (int i=0; i<q.size(); i++)
            qdq.data.push_back(q(i));
        for (int i=0; i<dq.size(); i++)
            qdq.data.push_back(dq(i));
        refqdq_pub.publish(qdq);
    }

    void drawRefPath(void)
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
        
        for (int i=0; i<T; i++)
        {
            pt.x = xref(0, i);
            pt.y = xref(1, i);
            pt.z = 0.0;
            line_strip.points.push_back(pt);
        }
        ref_pub.publish(line_strip);
    }

    bool atGoal(void) { return at_goal; }

public:
	OMPC() {}
    void init(ros::NodeHandle &nh);
	~OMPC() {}
};