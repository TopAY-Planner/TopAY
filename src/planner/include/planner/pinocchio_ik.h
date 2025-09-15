#pragma once

#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

class PinocchioIK
{
private:
    pinocchio::Model model;
    pinocchio::Data data;
    int JOINT_ID = 7;
    double eps = 1e-4;
    int IT_MAX = 10000;
    double DT = 1e-2;
    double damp = 1e-12;

public:
    void init(ros::NodeHandle& nh);
    std::pair<bool, Eigen::VectorXd> solveIK(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
};