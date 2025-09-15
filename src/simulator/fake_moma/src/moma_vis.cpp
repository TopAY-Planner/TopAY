#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fake_moma/MomaState.h"
#include "fake_moma/moma_param.h"

using namespace std;

ros::Subscriber state_sub;
ros::Publisher marker_pub, cylinder_pub, sphere_pub;

visualization_msgs::MarkerArray moma_marker;
visualization_msgs::MarkerArray colli_marker;

Eigen::Vector3d now_se2 = Eigen::Vector3d::Zero();
vector<float> now_q;
MomaParam moma_param;

Eigen::Quaterniond euler2rotation(double r, double p, double y)
{
	return Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()) 
			* Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) 
			* Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ());
}

Eigen::Quaterniond euler2rotation(Eigen::Vector3d rpy)
{
	return Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX()) 
			* Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) 
			* Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
}

void initParams()
{
	//diff_marker
	visualization_msgs::Marker diff_marker;
	diff_marker.header.frame_id = "world";
	diff_marker.id = 0;
	diff_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	diff_marker.action = visualization_msgs::Marker::ADD;
	diff_marker.pose.orientation.w = 1.0;
	diff_marker.pose.position.z = moma_param.chassis_height;
	Eigen::Quaterniond q = euler2rotation(M_PI_2, 0.0, 0.0);
	diff_marker.pose.orientation.w = q.w();
	diff_marker.pose.orientation.x = q.x();
	diff_marker.pose.orientation.y = q.y();
	diff_marker.pose.orientation.z = q.z();
	diff_marker.color.a = 1.0;
	diff_marker.color.r = 0.5;
	diff_marker.color.g = 0.5;
	diff_marker.color.b = 0.5;
	diff_marker.scale.x = 1.0;
	diff_marker.scale.y = 1.0;
	diff_marker.scale.z = 1.0;
	diff_marker.mesh_resource = "package://fake_moma/meshes/tracer.dae";
	moma_marker.markers.push_back(diff_marker);

	Eigen::Vector3d ap(diff_marker.pose.position.x, diff_marker.pose.position.y, diff_marker.pose.position.z);
	Eigen::Quaterniond aq(1.0, 0.0, 0.0, 0.0);
	ap += aq.matrix() * moma_param.relative_t;
	aq = aq.matrix() * moma_param.relative_R;

	//link0
	visualization_msgs::Marker link_marker;
	link_marker.header.frame_id = "world";
	link_marker.id = 1;
	link_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	link_marker.action = visualization_msgs::Marker::ADD;
	link_marker.pose.position.x = ap.x();
	link_marker.pose.position.y = ap.y();
	link_marker.pose.position.z = ap.z();
	link_marker.pose.orientation.w = aq.w();
	link_marker.pose.orientation.x = aq.x();
	link_marker.pose.orientation.y = aq.y();
	link_marker.pose.orientation.z = aq.z();
	link_marker.color.a = 1.0;
	link_marker.color.r = 0.5;
	link_marker.color.g = 0.5;
	link_marker.color.b = 0.5;
	link_marker.scale.x = 1.0;
	link_marker.scale.y = 1.0;
	link_marker.scale.z = 1.0;
	link_marker.mesh_resource = "package://fake_moma/meshes/link0.STL";
	moma_marker.markers.push_back(link_marker);

	//link1-7
	for (size_t i = 0; i < moma_param.dof_num; i++)
	{
		now_q.push_back(0.0);
		visualization_msgs::Marker link_marker;
		link_marker.header.frame_id = "world";
		link_marker.id = i+2;
		link_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		link_marker.action = visualization_msgs::Marker::ADD;
		ap += aq.matrix() * Eigen::Vector3d(0.0, 0.0, moma_param.link_length[i]);
		link_marker.pose.position.x = ap.x();
		link_marker.pose.position.y = ap.y();
		link_marker.pose.position.z = ap.z();
		aq = aq.matrix() * euler2rotation(moma_param.joint_offset.row(i));
		link_marker.pose.orientation.w = aq.w();
		link_marker.pose.orientation.x = aq.x();
		link_marker.pose.orientation.y = aq.y();
		link_marker.pose.orientation.z = aq.z();
		link_marker.color.a = 1.0;
		link_marker.color.r = 0.5;
		link_marker.color.g = 0.5;
		link_marker.color.b = 0.5;
		link_marker.scale.x = 1.0;
		link_marker.scale.y = 1.0;
		link_marker.scale.z = 1.0;
		link_marker.mesh_resource = "package://fake_moma/meshes/link"+std::to_string(i+1)+".STL";
		moma_marker.markers.push_back(link_marker);
	}

	//gripper
	visualization_msgs::Marker gripper_marker;
	gripper_marker.header.frame_id = "world";
	gripper_marker.id = 9;
	gripper_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	gripper_marker.action = visualization_msgs::Marker::ADD;
	gripper_marker.pose = moma_marker.markers.back().pose;
	gripper_marker.color.a = 1.0;
	gripper_marker.color.r = 0.5;
	gripper_marker.color.g = 0.5;
	gripper_marker.color.b = 0.5;
	gripper_marker.scale.x = 1.0;
	gripper_marker.scale.y = 1.0;
	gripper_marker.scale.z = 1.0;
	gripper_marker.mesh_resource = "package://fake_moma/meshes/gripper.dae";
	moma_marker.markers.push_back(gripper_marker);
}

double normyaw(const double& yaw)
{
	double y = yaw;
	if (y > M_PI)
		y-=2*M_PI;
	else if (y < -M_PI)
		y+=2*M_PI;
	return y;
}

void rcvStateCallBack(const fake_moma::MomaStatePtr msg)
{
	// chassis
	now_se2[0] = msg->chassis_odom.pose.pose.position.x;
	now_se2[1] = msg->chassis_odom.pose.pose.position.y;
	double ori_z = msg->chassis_odom.pose.pose.orientation.z;
	double ori_w = msg->chassis_odom.pose.pose.orientation.w;
	now_se2[2] = atan2(2.0*ori_z*ori_w, 
						2.0*ori_w*ori_w-1.0);
	Eigen::Quaterniond q = euler2rotation(M_PI_2, now_se2(2), 0.0);
	moma_marker.markers[0].header.stamp = ros::Time::now();
	moma_marker.markers[0].pose = msg->chassis_odom.pose.pose;
	moma_marker.markers[0].pose.position.z = moma_param.chassis_height;
	moma_marker.markers[0].pose.orientation.w = q.w();
	moma_marker.markers[0].pose.orientation.x = q.x();
	moma_marker.markers[0].pose.orientation.y = q.y();
	moma_marker.markers[0].pose.orientation.z = q.z();

	// arm
	// now_q = moma_cmd.q.data;
	for (size_t i=0; i<moma_param.dof_num; i++)
		now_q[i] = msg->arm_odom[i].twist.twist.linear.x;
	
	moma_marker.markers[1].pose = moma_marker.markers[0].pose;
	Eigen::Vector3d ap(moma_marker.markers[0].pose.position.x, 
					   moma_marker.markers[0].pose.position.y, 
					   moma_marker.markers[0].pose.position.z);
	Eigen::Quaterniond aq(cos(now_se2(2)/2.0), 0.0, 0.0, sin(now_se2(2)/2.0));
	ap += aq.matrix() * moma_param.relative_t;
	aq = aq.matrix() * moma_param.relative_R;

	moma_marker.markers[1].pose.position.x = ap.x();
	moma_marker.markers[1].pose.position.y = ap.y();
	moma_marker.markers[1].pose.position.z = ap.z();
	moma_marker.markers[1].pose.orientation.w = aq.w();
	moma_marker.markers[1].pose.orientation.x = aq.x();
	moma_marker.markers[1].pose.orientation.y = aq.y();
	moma_marker.markers[1].pose.orientation.z = aq.z();

	for (size_t i=0; i<moma_param.dof_num; i++)
	{
		now_q[i] = std::max( moma_param.joint_pos_limit_min[i], 
				   std::min( moma_param.joint_pos_limit_max[i], (double) (now_q[i]) ) );
		ap += aq.matrix() * Eigen::Vector3d(0.0, 0.0, moma_param.link_length[i]);
		aq = aq.matrix() * euler2rotation(moma_param.joint_offset.row(i))
						* euler2rotation(moma_param.joint_dof_axis.row(i)*now_q[i]);
		moma_marker.markers[2+i].pose.position.x = ap.x();
		moma_marker.markers[2+i].pose.position.y = ap.y();
		moma_marker.markers[2+i].pose.position.z = ap.z();
		moma_marker.markers[2+i].pose.orientation.w = aq.w();
		moma_marker.markers[2+i].pose.orientation.x = aq.x();
		moma_marker.markers[2+i].pose.orientation.y = aq.y();
		moma_marker.markers[2+i].pose.orientation.z = aq.z();
	}

	// gripper
	moma_marker.markers.back().pose = moma_marker.markers[8].pose;

	// collision detection
	Eigen::VectorXd moma_pos(3+moma_param.dof_num);
	moma_pos.head(3) = now_se2;
	for (size_t i=0; i<moma_param.dof_num; i++)
		moma_pos[3+i] = now_q[i];
	
	Eigen::VectorXi collision_link;
	moma_param.isSelfCollision(moma_pos, collision_link);

	visualization_msgs::MarkerArray cylinder_msg = moma_param.getColliCylinderArray(moma_pos);
	marker_pub.publish(moma_marker);
	cylinder_pub.publish(cylinder_msg);
	auto cma = moma_param.getColliMarkerArray(moma_pos);
	for (size_t i=0; i<cma.markers.size(); i++)
		cma.markers[i].header.stamp = ros::Time::now();
	sphere_pub.publish(cma);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "moma_vis_node");
    ros::NodeHandle nh( "~" );

	initParams();
    state_sub  = nh.subscribe("state", 1, rcvStateCallBack);
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
	cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinder", 1);
	sphere_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere", 1);

	ros::spin();
    return 0;
}