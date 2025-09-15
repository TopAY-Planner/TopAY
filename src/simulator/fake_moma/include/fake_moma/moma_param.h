#pragma once

#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <vector>

#define PRINTF_WHITE(STRING) std::cout<<STRING
#define PRINT_GREEN(STRING) std::cout<<"\033[92m"<<STRING<<"\033[m\n"
#define PRINT_RED(STRING) std::cout<<"\033[31m"<<STRING<<"\033[m\n"
#define PRINTF_RED(STRING) std::cout<<"\033[31m"<<STRING<<"\033[m"
#define PRINT_YELLOW(STRING) std::cout<<"\033[33m"<<STRING<<"\033[m\n"
#define PRINTF_YELLOW(STRING) std::cout<<"\033[33m"<<STRING<<"\033[m"

// Same as the original NodeHandle::getParam, but throws exception if the parameter is not found
#define GET_PARAM_OR_THROW(nh, param_name, param_var) \
    do { \
        if (!(nh).getParam((param_name), (param_var))) { \
            std::string error_msg = "Failed to get required parameter: " + std::string(param_name); \
            throw std::runtime_error(error_msg); \
        } \
    } while (0)

using namespace Eigen;
using namespace std;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

struct MomaParam
{
    // chassis parameters
    double chassis_length = 0.685;
    double chassis_width = 0.57;
    double chassis_height = 0.155;
    double chassis_colli_radius = 0.4;
    double max_v = 1.0;
    double max_a = 0.8;
    double max_w = 1.25; // for equally compare with zzd
    // double max_w = 0.9;
    double max_dw = 1.0;
    //! TEST ZMK
    // double max_v = 3.0;
    // double max_a = 2.0;
    // double max_w = 4.0;
    // double max_dw = 6.0;
    //! TEST ZMK

    // arm parameters
    size_t dof_num = 7;
    double cylinder_radius = 0.055;
    VectorXd colli_length;
    VectorXd colli_points;
    VectorXd colli_point_radius;
    VectorXd default_colli_point_radius;
    VectorXi colli_link_map;
    VectorXd link_length;
    VectorXd joint_pos_limit_min;
    VectorXd joint_pos_limit_max;
    VectorXd joint_vel_limit;
    VectorXd joint_acc_limit;
    VectorXd joint_eff_limit;
    MatrixX3d joint_offset;
    MatrixX3d joint_dof_axis;
    MatrixXi collision_matrix;
    Matrix3d relative_R;
    Vector3d relative_t;

    MomaParam()
    {
        link_length = joint_pos_limit_min = joint_acc_limit = \
        joint_pos_limit_max = joint_vel_limit = joint_eff_limit = VectorXd::Zero(dof_num);
        joint_offset = joint_dof_axis = MatrixX3d::Zero(dof_num, 3);
        joint_offset << -1.5708, 0.0, 0.0,
                       1.5708, 0.0, 0.0,
                       -1.5708, 0.0, 0.0,
                       1.5708, 0.0, 0.0,
                       -1.5708, 0.0, 0.0,
                       1.5708, 0.0, 0.0,
                       0.0, 0.0, 0.0;
        joint_dof_axis << 0.0, -1.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, -1.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, -1.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0;
        colli_length = VectorXd::Zero(dof_num+1);
        colli_length << 0.139, 0.1015, 0.1525, 0.1035, 0.1285, 0.0815, 0.144, 0.05;
        colli_point_radius = colli_points = VectorXd::Zero((dof_num+1)*2);
        colli_points << 0.139-0.09, 0.139, 
                        0.0, 0.1015, 
                        0.1525-0.08, 0.1525,
                        0.0, 0.1035, 
                        0.1285-0.07, 0.1285, 
                        0.0, 0.0815, 
                        0.144-0.07, 0.144,
                        0.0, 0.1;
        colli_point_radius << 0.06, 0.06, 
                              0.0, 0.08, 
                              0.04, 0.04, 
                              0.0, 0.07, 
                              0.035, 0.035, 
                              0.0, 0.06, 
                              0.035, 0.035,
                              0.0, 0.08;
        for (int i=0; i<colli_point_radius.size(); i++)
            if (colli_point_radius(i) > 1e-4 && colli_point_radius(i) < cylinder_radius)
                colli_point_radius(i) = cylinder_radius;
        default_colli_point_radius = colli_point_radius;
        link_length << 0.2405, 0.0, 0.256, 0.0, 0.21, 0.0, 0.144;
        joint_pos_limit_min << -3.1, -2.26, -3.1, -2.355, -3.1, -2.23, -6.28;
        joint_pos_limit_max << 3.1, 2.26, 3.1, 2.355, 3.1, 2.23, 6.28;
        //TODO: read from realman
        joint_vel_limit.setConstant(2.35);
        // joint_vel_limit << 3.14, 3.14, 3.92, 3.92, 3.92, 3.92, 3.92;
        joint_acc_limit.setConstant(6.28);
        joint_eff_limit << 60.0, 60.0, 30.0, 30.0, 10.0, 10.0, 10.0;

        relative_R << 0.7071068, 0.7071068, 0.0,
                      -0.7071068, 0.7071068, 0.0,
                      0.0, 0.0, 1.0;
        relative_t << 0.0, 0.115, 0.016;
        
        std::vector<Eigen::Vector4d> cpts = getColliPts(VectorXd::Zero(3+dof_num));
        colli_link_map.resize(cpts.size());
        colli_link_map << 0, 0, 1, 2, 2, 3, 4, 4, 5, 6, 6, 7;
        collision_matrix.resize(cpts.size(), cpts.size());
        collision_matrix.setConstant(-1);
        for (size_t i=0; i<cpts.size(); i++)
        {
            for (size_t j=i; j<cpts.size(); j++)
            {
                if (i == j)
                    collision_matrix(i, j) = 1;
                double dist = (cpts[i].head(3) - cpts[j].head(3)).norm();
                if (dist < cpts[i][3] + cpts[j][3])
                    collision_matrix(i, j) = collision_matrix(j, i) = 1;
            }
        }
    }

    void setColliRs(const Eigen::VectorXd &colli_rs)
    {
        colli_point_radius.setZero();
        colli_point_radius << colli_rs(0), colli_rs(1), 
                                0.0, colli_rs(2),
                                colli_rs(3), colli_rs(4), 
                                0.0, colli_rs(5), 
                                colli_rs(6), colli_rs(7), 
                                0.0, colli_rs(8), 
                                colli_rs(9), colli_rs(10),
                                0.0, colli_rs(11);
        chassis_colli_radius = colli_rs(12);
        return;
    }

    void resetColliRs()
    {
        colli_point_radius = default_colli_point_radius;
        chassis_colli_radius = 0.4;
        return;
    }

    bool isSelfCollision(const Eigen::VectorXd& moma_pos, Eigen::VectorXi& collision_link)
    {
        collision_link.resize(2+dof_num);
        collision_link.setZero();

        bool is_collision = false;
        std::vector<Eigen::Vector4d> cpts = getColliPts(moma_pos);
        for (size_t i=0; i<cpts.size(); i++)
        {
            if (i!=0 && cpts[i](2) < chassis_height + cpts[i](3) + relative_t(2)
                // && (cpts[i].head(2) - moma_pos.head(2)).norm() < chassis_colli_radius + cpts[i](3)
                )
            {
                collision_link(colli_link_map(i)+1) = 1;
                collision_link(0) = 1;
                is_collision = true;
            }

            for (size_t j=i; j<cpts.size(); j++)
            {
                if (i == j)
                    continue;
                double dist = (cpts[i].head(3) - cpts[j].head(3)).norm();
                if (dist < cpts[i][3] + cpts[j][3] - 1e-2 && collision_matrix(i, j) == -1)
                {
                    collision_link(colli_link_map(i)+1) = 1;
                    collision_link(colli_link_map(j)+1) = 1;
                    is_collision = true;
                }
            }
        }
        
        return is_collision;
    }

    std::vector<Eigen::Vector4d> getColliPts(const Eigen::VectorXd& moma_pos) const
    {
        std::vector<Eigen::Vector4d> colli_pts;
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        for (size_t i = 0; i < dof_num + 1; i++)
        {
            for (int j=0; j<2; j++)
            {
                if (colli_points[i*2+j] == 0.0)
                    continue;
                Eigen::Vector3d now_colli_p = now_p + now_R.col(2) * colli_points[i*2+j];
                Eigen::Vector4d colli_pt;
                colli_pt.head(3) = now_colli_p;
                colli_pt[3] = colli_point_radius[i*2+j];
                colli_pts.push_back(colli_pt);
            }
            
            now_p += now_R.col(2) * colli_length[i];
            double pn = 1.0;
            Eigen::Matrix3d dof_R;
            if (i == dof_num)
                break;
            if (i % 2 == 0)
            {
                dof_R << cos(pn*moma_pos[3+i]), -sin(pn*moma_pos[3+i]), 0.0,
                         sin(pn*moma_pos[3+i]), cos(pn*moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
            }
            else
            {
                dof_R << cos(pn*moma_pos[3+i]), 0.0, sin(pn*moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(pn*moma_pos[3+i]), 0.0, cos(pn*moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
        }

        return colli_pts;
    }

    Eigen::VectorXd getColliGrads(const Eigen::VectorXd& moma_pos,
                                  const std::vector<Eigen::Vector3d>& pos_grads)
    {
        int gidx = 0;
        Eigen::VectorXd colli_grads = Eigen::VectorXd::Zero(3+dof_num);
        Eigen::MatrixXd grad_p_list = Eigen::MatrixXd::Zero(3, dof_num+1);
        Eigen::MatrixXd grad_R_list = Eigen::MatrixXd::Zero(3, dof_num+1);
        std::vector<Eigen::Matrix3d> now_R_list;
        std::vector<Eigen::Matrix3d> R_list;
        std::vector<Eigen::Matrix3d> dR_list;
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        R_list.push_back(now_R);
        Eigen::Matrix3d dR;
        dR << -sin(moma_pos[2]), -cos(moma_pos[2]), 0.0,
                cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                0.0, 0.0, 0.0;
        dR = dR * relative_R;
        dR_list.push_back(dR);
        for (size_t i = 0; i < dof_num + 1; i++)
        {
            now_R_list.push_back(now_R);
            for (int j=0; j<2; j++)
            {
                if (colli_points[i*2+j] == 0.0)
                    continue;
                grad_p_list.col(i) += pos_grads[gidx];
                grad_R_list.col(i) += colli_points[i*2+j] * pos_grads[gidx];
                gidx++;
            }
            if (i == dof_num)
                break;

            now_p += now_R.col(2) * colli_length[i];
            Eigen::Matrix3d dof_R;
            if (i % 2 == 0)
            {
                dof_R << cos(moma_pos[3+i]), -sin(moma_pos[3+i]), 0.0,
                         sin(moma_pos[3+i]), cos(moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
                dR << -sin(moma_pos[3+i]), -cos(moma_pos[3+i]), 0.0,
                        cos(moma_pos[3+i]), -sin(moma_pos[3+i]), 0.0,
                        0.0, 0.0, 0.0;
            }
            else
            {
                dof_R << cos(moma_pos[3+i]), 0.0, sin(moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(moma_pos[3+i]), 0.0, cos(moma_pos[3+i]);
                dR << -sin(moma_pos[3+i]), 0.0, cos(moma_pos[3+i]),
                      0.0, 0.0, 0.0,
                      -cos(moma_pos[3+i]), 0.0, -sin(moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
            R_list.push_back(dof_R);
            dR_list.push_back(dR);
        }

        for (int i = dof_num; i > 0; i--)
        {
            colli_grads(2+i) += (now_R_list[i-1] * dR_list[i].col(2)).dot(grad_R_list.col(i));
            for (int j=0; j<i; j++)
            {
                Eigen::Matrix3d dR = dR_list[j];
                Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
                for (int k=0; k<j; k++)
                    R1 = R1 * R_list[k];
                R1 = R1 * dR;
                for (int k=j+1; k<=i; k++)
                    R1 = R1 * R_list[k];
                colli_grads(2+j) += R1.col(2).dot(grad_R_list.col(i));
            }
            grad_R_list.col(i-1) += grad_p_list.col(i) * colli_length[i-1];
            grad_p_list.col(i-1) += grad_p_list.col(i);
        }
        colli_grads(2) += dR_list[0].col(2).dot(grad_R_list.col(0));
        dR << -sin(moma_pos[2]), -cos(moma_pos[2]), 0.0,
                cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                0.0, 0.0, 0.0;
        colli_grads(2) += (dR * relative_t).dot(grad_p_list.col(0));
        colli_grads.head(2) = grad_p_list.col(0).head(2);

        return colli_grads;
    }

    Eigen::VectorXd getFKPose(const Eigen::VectorXd& moma_pos)
    {
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        for (size_t i = 0; i < dof_num; i++)
        {
            now_p += now_R.col(2) * colli_length[i];
            double pn = 1.0;
            Eigen::Matrix3d dof_R;
            if (i % 2 == 0)
            {
                dof_R << cos(pn*moma_pos[3+i]), -sin(pn*moma_pos[3+i]), 0.0,
                         sin(pn*moma_pos[3+i]), cos(pn*moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
            }
            else
            {
                dof_R << cos(pn*moma_pos[3+i]), 0.0, sin(pn*moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(pn*moma_pos[3+i]), 0.0, cos(pn*moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
        }
        Eigen::VectorXd fk_pose = Eigen::VectorXd::Zero(3+6);
        fk_pose.head(3) = now_p;
        fk_pose.segment(3, 3) = now_R.row(0);
        fk_pose.tail(3) = now_R.row(1);

        return fk_pose;
    }

    Eigen::VectorXd getEEGrads(const Eigen::VectorXd& moma_pos,
                              const Eigen::VectorXd& ee_grad)
    {
        int gidx = 0;
        Eigen::VectorXd moma_grads = Eigen::VectorXd::Zero(3+dof_num);
        Eigen::MatrixXd grad_p_list = Eigen::MatrixXd::Zero(3, dof_num+1);
        Eigen::MatrixXd grad_R_list = Eigen::MatrixXd::Zero(3, dof_num+1);
        std::vector<Eigen::Matrix3d> now_R_list;
        std::vector<Eigen::Matrix3d> R_list;
        std::vector<Eigen::Matrix3d> dR_list;
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        R_list.push_back(now_R);
        Eigen::Matrix3d dR;
        dR << -sin(moma_pos[2]), -cos(moma_pos[2]), 0.0,
                cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                0.0, 0.0, 0.0;
        dR = dR * relative_R;
        dR_list.push_back(dR);
        for (size_t i = 0; i < dof_num; i++)
        {
            now_R_list.push_back(now_R);
            now_p += now_R.col(2) * colli_length[i];

            Eigen::Matrix3d dof_R;
            if (i % 2 == 0)
            {
                dof_R << cos(moma_pos[3+i]), -sin(moma_pos[3+i]), 0.0,
                         sin(moma_pos[3+i]), cos(moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
                dR << -sin(moma_pos[3+i]), -cos(moma_pos[3+i]), 0.0,
                        cos(moma_pos[3+i]), -sin(moma_pos[3+i]), 0.0,
                        0.0, 0.0, 0.0;
            }
            else
            {
                dof_R << cos(moma_pos[3+i]), 0.0, sin(moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(moma_pos[3+i]), 0.0, cos(moma_pos[3+i]);
                dR << -sin(moma_pos[3+i]), 0.0, cos(moma_pos[3+i]),
                      0.0, 0.0, 0.0,
                      -cos(moma_pos[3+i]), 0.0, -sin(moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
            R_list.push_back(dof_R);
            dR_list.push_back(dR);
        }

        grad_p_list.col(dof_num) = ee_grad.head(3);
        for (int i = dof_num; i > 0; i--)
        {
            moma_grads(2+i) += (now_R_list[i-1] * dR_list[i].col(2)).dot(grad_R_list.col(i));
            for (int j=0; j<i; j++)
            {
                Eigen::Matrix3d dR = dR_list[j];
                Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
                for (int k=0; k<j; k++)
                    R1 = R1 * R_list[k];
                R1 = R1 * dR;
                for (int k=j+1; k<=i; k++)
                    R1 = R1 * R_list[k];
                moma_grads(2+j) += R1.col(2).dot(grad_R_list.col(i));
            }
            grad_R_list.col(i-1) += grad_p_list.col(i) * colli_length[i-1];
            grad_p_list.col(i-1) += grad_p_list.col(i);
        }
        moma_grads(2) += dR_list[0].col(2).dot(grad_R_list.col(0));
        dR << -sin(moma_pos[2]), -cos(moma_pos[2]), 0.0,
                cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                0.0, 0.0, 0.0;
        moma_grads(2) += (dR * relative_t).dot(grad_p_list.col(0));
        moma_grads.head(2) = grad_p_list.col(0).head(2);

        // grad Ree
        for (int j=0; j<dof_num+1; j++)
        {
            Eigen::Matrix3d dR = dR_list[j];
            Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
            for (int k=0; k<j; k++)
                R1 = R1 * R_list[k];
            R1 = R1 * dR;
            for (int k=j+1; k<=dof_num; k++)
                R1 = R1 * R_list[k];
            moma_grads(2+j) += R1.row(0).dot(ee_grad.segment(3, 3));
            moma_grads(2+j) += R1.row(1).dot(ee_grad.tail(3));
        }

        return moma_grads;
    }

    visualization_msgs::MarkerArray getColliMarkerArray(Eigen::VectorXd moma_pos)
    {
        visualization_msgs::MarkerArray colli_marker_array;
        visualization_msgs::Marker chassis_marker;
        chassis_marker.header.frame_id = "world";
        chassis_marker.id = 0;
        chassis_marker.type = visualization_msgs::Marker::CYLINDER;
        chassis_marker.action = visualization_msgs::Marker::ADD;
        chassis_marker.scale.x = chassis_colli_radius * 2.0;
        chassis_marker.scale.y = chassis_colli_radius * 2.0;
        chassis_marker.scale.z = chassis_height;
        chassis_marker.pose.position.x = moma_pos[0];
        chassis_marker.pose.position.y = moma_pos[1];
        chassis_marker.pose.position.z =  chassis_height / 2.0;
        chassis_marker.pose.orientation.w = 1.0;
        chassis_marker.color.a = 0.5;
        chassis_marker.color.r = 0.0;
        chassis_marker.color.g = 0.0;
        chassis_marker.color.b = 1.0;
        colli_marker_array.markers.push_back(chassis_marker);
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        for (size_t i = 0; i < dof_num + 1; i++)
        {
            for (int j=0; j<2; j++)
            {
                if (colli_points[i*2+j] == 0.0)
                    continue;
                Eigen::Vector3d now_colli_p = now_p + now_R.col(2) * colli_points[i*2+j];
                visualization_msgs::Marker colli_marker;
                colli_marker.header.frame_id = "world";
                colli_marker.id = i*2+j+1;
                colli_marker.type = visualization_msgs::Marker::SPHERE;
                colli_marker.action = visualization_msgs::Marker::ADD;
                colli_marker.scale.x = colli_point_radius[i*2+j] * 2.0;
                colli_marker.scale.y = colli_point_radius[i*2+j] * 2.0;
                colli_marker.scale.z = colli_point_radius[i*2+j] * 2.0;
                colli_marker.pose.position.x = now_colli_p[0];
                colli_marker.pose.position.y = now_colli_p[1];
                colli_marker.pose.position.z = now_colli_p[2];
                colli_marker.pose.orientation.w = 1.0;
                colli_marker.color.a = 0.5;
                colli_marker.color.r = 0.0;
                colli_marker.color.g = 0.0;
                colli_marker.color.b = 1.0;
                colli_marker_array.markers.push_back(colli_marker);
            }
            
            now_p += now_R.col(2) * colli_length[i];
            double pn = 1.0;
            Eigen::Matrix3d dof_R;
            if (i == dof_num)
                break;
            if (i % 2 == 0)
            {
                dof_R << cos(pn*moma_pos[3+i]), -sin(pn*moma_pos[3+i]), 0.0,
                         sin(pn*moma_pos[3+i]), cos(pn*moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
            }
            else
            {
                dof_R << cos(pn*moma_pos[3+i]), 0.0, sin(pn*moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(pn*moma_pos[3+i]), 0.0, cos(pn*moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
        }
        return colli_marker_array;
    }

    visualization_msgs::MarkerArray getColliCylinderArray(Eigen::VectorXd moma_pos)
    {
        visualization_msgs::MarkerArray colli_marker_array;
        visualization_msgs::Marker chassis_marker;
        chassis_marker.header.frame_id = "world";
        chassis_marker.header.stamp = ros::Time::now();
        chassis_marker.id = 0;
        chassis_marker.type = visualization_msgs::Marker::CYLINDER;
        chassis_marker.action = visualization_msgs::Marker::ADD;
        chassis_marker.scale.x = chassis_colli_radius * 2.0;
        chassis_marker.scale.y = chassis_colli_radius * 2.0;
        chassis_marker.scale.z = chassis_height;
        chassis_marker.pose.position.x = moma_pos[0];
        chassis_marker.pose.position.y = moma_pos[1];
        chassis_marker.pose.position.z = chassis_height / 2.0;
        chassis_marker.pose.orientation.w = 1.0;
        chassis_marker.color.a = 0.5;
        chassis_marker.color.r = 0.0;
        chassis_marker.color.g = 0.0;
        chassis_marker.color.b = 1.0;
        colli_marker_array.markers.push_back(chassis_marker);
        Eigen::Vector3d now_p(moma_pos[0], moma_pos[1], chassis_height);
        Eigen::Matrix3d now_R;
        now_R << cos(moma_pos[2]), -sin(moma_pos[2]), 0.0,
                 sin(moma_pos[2]), cos(moma_pos[2]), 0.0,
                 0.0, 0.0, 1.0;
        now_p += now_R * relative_t;
        now_R = now_R * relative_R;
        for (size_t i = 0; i < dof_num; i++)
        {
            Eigen::Vector3d now_colli_p = now_p + now_R.col(2) * colli_length[i] / 2.0;
            Eigen::Quaterniond now_colli_q(now_R);
            visualization_msgs::Marker colli_marker;
            colli_marker.header.frame_id = "world";
            colli_marker.header.stamp = ros::Time::now();
            colli_marker.id = i+1;
            colli_marker.type = visualization_msgs::Marker::CYLINDER;
            colli_marker.action = visualization_msgs::Marker::ADD;
            colli_marker.scale.x = cylinder_radius * 2.0;
            colli_marker.scale.y = cylinder_radius * 2.0;
            colli_marker.scale.z = colli_length[i];
            colli_marker.pose.position.x = now_colli_p[0];
            colli_marker.pose.position.y = now_colli_p[1];
            colli_marker.pose.position.z = now_colli_p[2];
            colli_marker.pose.orientation.w = now_colli_q.w();
            colli_marker.pose.orientation.x = now_colli_q.x();
            colli_marker.pose.orientation.y = now_colli_q.y();
            colli_marker.pose.orientation.z = now_colli_q.z();
            colli_marker.color.a = 0.5;
            colli_marker.color.r = 0.0;
            colli_marker.color.g = 0.0;
            colli_marker.color.b = 1.0;
            colli_marker_array.markers.push_back(colli_marker);
            now_p += now_R.col(2) * colli_length[i];
            double pn = 1.0;
            // double pn = joint_dof_axis(i, 1);
            Eigen::Matrix3d dof_R;
            if (i % 2 == 0)
            {
                dof_R << cos(pn*moma_pos[3+i]), -sin(pn*moma_pos[3+i]), 0.0,
                         sin(pn*moma_pos[3+i]), cos(pn*moma_pos[3+i]), 0.0,
                         0.0, 0.0, 1.0;
            }
            else
            {
                dof_R << cos(pn*moma_pos[3+i]), 0.0, sin(pn*moma_pos[3+i]),
                         0.0, 1.0, 0.0,
                         -sin(pn*moma_pos[3+i]), 0.0, cos(pn*moma_pos[3+i]);
            }
            now_R = now_R * dof_R;
        }
        return colli_marker_array;
    }

    Eigen::VectorXd getJointLimitsMin()
    {
        return joint_pos_limit_min;
    }

    Eigen::VectorXd getJointLimitsMax()
    {
        return joint_pos_limit_max;
    }

    Eigen::VectorXd getJointVelLimits()
    {
        return joint_vel_limit;
    }

    int toAddress(const Eigen::Vector3i& voxel_num, const Eigen::Vector3i& id)
    {
        return id(0) * voxel_num(1) * voxel_num(2) + id(1) * voxel_num(2) + id(2);
    }

    int toAddress(const Eigen::Vector3i& voxel_num, int& x, int& y, int& z) 
    {
        return x * voxel_num(1) * voxel_num(2) + y * voxel_num(2) + z;
    }

    void getBallsGrids(std::unordered_map<std::string, double> params, 
                        const Eigen::VectorXd& obstacle_data,
                        Eigen::Ref<Eigen::VectorXi> result)
    {
        clock_t start = clock();

        int             buffer_size;
        double          resolution, resolution_inv;
        Eigen::Vector3d map_origin;
        Eigen::Vector3d map_size;
        Eigen::Vector3i min_idx;
        Eigen::Vector3i max_idx;
        Eigen::Vector3i voxel_num;

        map_size[0] = params["map_size_x"];
        map_size[1] = params["map_size_y"];
        map_size[2] = params["map_size_z"];
        resolution = params["resolution"];
        map_origin[0] = params["origin_x"];
        map_origin[1] = params["origin_y"];
        map_origin[2] = params["origin_z"];
        
        // std::cout<<"resolution: "<<resolution<<std::endl;
        // std::cout<<"map_size: "<<map_size.transpose()<<std::endl;

        // origin and boundary

        // resolution
        resolution_inv = 1.0 / resolution;

        // voxel num
        voxel_num(0) = ceil(map_size(0) / resolution);
        voxel_num(1) = ceil(map_size(1) / resolution);
        voxel_num(2) = ceil(map_size(2) / resolution);

        // idx
        min_idx = Eigen::Vector3i::Zero();
        max_idx = voxel_num - Eigen::Vector3i::Ones();

        // datas
        buffer_size  = voxel_num(0) * voxel_num(1) * voxel_num(2);
        result.resize(buffer_size);
        // std::cout<<result.size()<<std::endl;

        // set obstacles
        int osize = obstacle_data.size() / 4;
        for (size_t i = 0; i < osize; i++)
        {
            Eigen::Vector3i id;
            Eigen::Vector3d pos(obstacle_data(i*4), obstacle_data(i*4+1), obstacle_data(i*4+2));
            double r = obstacle_data(i*4+3);
            int p = ceil(r / resolution);
            for (int x = -p; x <= p; ++x)
            {
                for (int y = -p; y <= p; ++y)
                {
                    for (int z = -p; z <= p; ++z)
                    {
                        Eigen::Vector3d ps = pos + Eigen::Vector3d(x, y, z) * resolution;
                        if ((ps-pos).norm() <= r)
                        {
                            id(0) = std::max(min_idx(0), std::min(max_idx(0), (int)floor((ps(0) - map_origin(0)) * resolution_inv)));
                            id(1) = std::max(min_idx(1), std::min(max_idx(1), (int)floor((ps(1) - map_origin(1)) * resolution_inv)));
                            id(2) = std::max(min_idx(2), std::min(max_idx(2), (int)floor((ps(2) - map_origin(2)) * resolution_inv)));
                            if (id(0) >= min_idx(0) && id(0) <= max_idx(0) &&
                                id(1) >= min_idx(1) && id(1) <= max_idx(1) &&
                                id(2) >= min_idx(2) && id(2) <= max_idx(2))
                            {
                                int address = toAddress(voxel_num, id);
                                result[address] = 1;
                            }
                        }
                    }
                }
            }
        }

        return;
    }

    std::vector<Eigen::VectorXd> getMeshPose(const Eigen::VectorXd& moma_pos) const {

        std::vector<Eigen::VectorXd> ret;

        auto euler2rotation = [](double r, double p, double y) -> Eigen::Quaterniond {
            return Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) 
                * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) 
                * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
        };

        auto eulerV2rotation = [](Eigen::Vector3d rpy) -> Eigen::Quaterniond {
            return Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) 
                * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) 
                * Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
        };

        // TODO check height
        // chassis
        Eigen::VectorXd chassis_pose;
        chassis_pose.resize(7);
        {
            Eigen::Quaterniond q = euler2rotation(M_PI_2, moma_pos(2), 0.0);
            chassis_pose << moma_pos[0], moma_pos[1], chassis_height/2.0, q.w(), q.x(), q.y(), q.z();
        }
        ret.push_back(chassis_pose);
        
        // stump
        Eigen::Vector3d ap;
        ap = chassis_pose.head(3);
        Eigen::Quaterniond aq(cos(moma_pos(2)/2.0), 0.0, 0.0, sin(moma_pos(2)/2.0));

        ap += aq.matrix() * relative_t;
        aq = aq.matrix() * relative_R;

        Eigen::VectorXd stump_pose;
        stump_pose.resize(7);
        {
            stump_pose << ap.x() , ap.y() , ap.z() , aq.w() , aq.x() , aq.y() , aq.z();
        }
        ret.push_back(stump_pose);

        // links
        for (size_t i = 0; i < dof_num; i++){
            double q = moma_pos(3+i);
            q = std::max(joint_pos_limit_min[i], std::min(joint_pos_limit_max[i], q));
            ap += aq.matrix() * Eigen::Vector3d(0.0, 0.0, link_length[i]);
            aq = aq.matrix() * eulerV2rotation(joint_offset.row(i))
                            * eulerV2rotation(joint_dof_axis.row(i)*q);
            Eigen::VectorXd link_pose(7);
            link_pose << ap.x() , ap.y() , ap.z() , aq.w() , aq.x() , aq.y() , aq.z();
            ret.push_back(link_pose);
        }

        
        Eigen::VectorXd ee_pose(ret.back());
        ret.push_back(ee_pose);
        
        // ee
        Eigen::Vector4d ee_pt = getColliPts(moma_pos).back();
        Eigen::VectorXd ee_colli(7);
        ee_colli.setZero();
        ee_colli.head(3) = ee_pt.head(3);

        ret.push_back(ee_colli);

        return ret;
    }

};
