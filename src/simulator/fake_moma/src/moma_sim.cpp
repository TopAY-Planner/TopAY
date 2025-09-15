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
#include "fake_moma/MomaCmd.h"
#include "fake_moma/moma_param.h"

using namespace std;

ros::Subscriber cmd_sub, map_sub;
ros::Publisher  state_pub, marker_pub, cylinder_pub, sphere_pub, lidar_odom_pub;

fake_moma::MomaCmd moma_cmd;
fake_moma::MomaState moma_state;
nav_msgs::Odometry lidar_odom;
visualization_msgs::MarkerArray moma_marker;
visualization_msgs::MarkerArray colli_marker;

Eigen::Vector3d now_se2 = Eigen::Vector3d::Zero();
vector<float> now_q;
double time_resolution = 0.01;
bool rcv_cmd = false;
MomaParam moma_param;

bool has_map = false;
pcl::PointCloud<pcl::PointXYZ> cloud_map;
pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
pcl::PointXYZ sensor_pose;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

vector<Eigen::Vector2d> vw_buff;
double time_delay = 0.4;
// double time_delay = 0.4;
ros::Time get_cmdtime;

double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;

double lidar_x = 0.0;
double lidar_y = 0.0;
double lidar_z = 0.0;
Eigen::Vector3d lidar_vec;

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

void initParams(ros::NodeHandle& nh)
{
	nh.getParam("sim/init_x", init_x);
	nh.getParam("sim/init_y", init_y);
	nh.getParam("sim/init_yaw", init_yaw);
	nh.getParam("sim/lidar_x", lidar_x);
	nh.getParam("sim/lidar_y", lidar_y);
	nh.getParam("sim/lidar_z", lidar_z);

	now_se2 = Eigen::Vector3d(init_x, init_y, init_yaw);
	lidar_vec = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);

	//diff_state
	moma_state.chassis_odom.header.frame_id = "world";
	moma_state.chassis_odom.pose.pose.position.x = init_x;
	moma_state.chassis_odom.pose.pose.position.y = init_y;
	moma_state.chassis_odom.pose.pose.position.z = moma_param.chassis_height;
	moma_state.chassis_odom.pose.pose.orientation.w = cos(init_yaw/2.0);
	moma_state.chassis_odom.pose.pose.orientation.x = 0.0;
	moma_state.chassis_odom.pose.pose.orientation.y = 0.0;
	moma_state.chassis_odom.pose.pose.orientation.z = sin(init_yaw/2.0);
	moma_state.chassis_odom.twist.twist.linear.x = 0.0;
	moma_state.chassis_odom.twist.twist.linear.y = 0.0;
	moma_state.chassis_odom.twist.twist.linear.z = 0.0;
	moma_state.chassis_odom.twist.twist.angular.x = 0.0;
	moma_state.chassis_odom.twist.twist.angular.y = 0.0;
	moma_state.chassis_odom.twist.twist.angular.z = 0.0;

	// lidar odom
	Eigen::Quaterniond chasq(cos(init_yaw/2.0), 0.0, 0.0, sin(init_yaw/2.0));
	Eigen::Vector3d chasv(init_x, init_y, moma_param.chassis_height);
	Eigen::Vector3d lidar_pos = chasq.matrix() * lidar_vec + chasv;
	lidar_odom = moma_state.chassis_odom;
	lidar_odom.pose.pose.position.x = lidar_pos(0);
	lidar_odom.pose.pose.position.y = lidar_pos(1);
	lidar_odom.pose.pose.position.z = lidar_pos(2);

	//diff_marker
	visualization_msgs::Marker diff_marker;
	diff_marker.header.frame_id = "world";
	diff_marker.id = 0;
	diff_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	diff_marker.action = visualization_msgs::Marker::ADD;
	diff_marker.pose = moma_state.chassis_odom.pose.pose;
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
		moma_state.arm_odom.push_back(moma_state.chassis_odom);
		moma_state.arm_odom[i].pose.pose = link_marker.pose;
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

void rcvVelCmdCallBack(const fake_moma::MomaCmd& cmd)
{	
	moma_cmd = cmd;
	moma_cmd.speed = 0.0;
	moma_cmd.angular_velocity = 0.0;
	if (rcv_cmd==false)
	{
		rcv_cmd = true;
		vw_buff.push_back(Eigen::Vector2d(cmd.speed, cmd.angular_velocity));
		get_cmdtime = ros::Time::now();
	}
	else
	{
		vw_buff.push_back(Eigen::Vector2d(cmd.speed, cmd.angular_velocity));
		if ((ros::Time::now() - get_cmdtime).toSec() > time_delay)
		{
			moma_cmd.speed = vw_buff[0](0);
			moma_cmd.angular_velocity = vw_buff[0](1);
			vw_buff.erase(vw_buff.begin());
		}
	}
}

void rcvMapCallBack(const sensor_msgs::PointCloud2& msg)
{	
	if (has_map)
		return;
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	pcl::fromROSMsg(msg, cloud_map);
    kd_tree.setInputCloud(cloud_map.makeShared());
	has_map = true;
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

void simCallBack(const ros::TimerEvent& event)
{
	if (!rcv_cmd)
	{
		moma_state.chassis_odom.header.stamp = ros::Time::now();
    	state_pub.publish(moma_state);
		lidar_odom.header.stamp = ros::Time::now();
		lidar_odom_pub.publish(lidar_odom);
		moma_marker.markers[0].header.stamp = ros::Time::now();
		marker_pub.publish(moma_marker);
		Eigen::VectorXd moma_pos = VectorXd::Zero(3+moma_param.dof_num);
		moma_pos.head(3) = now_se2;
		cylinder_pub.publish(moma_param.getColliCylinderArray(moma_pos));
		auto cma = moma_param.getColliMarkerArray(moma_pos);
		for (size_t i=0; i<cma.markers.size(); i++)
			cma.markers[i].header.stamp = ros::Time::now();
		sphere_pub.publish(cma);
		return;
	}

	// chassis
	double vx = moma_cmd.speed;
	double wz = moma_cmd.angular_velocity;
	now_se2(0) += vx*time_resolution*cos(now_se2(2));
	now_se2(1) += vx*time_resolution*sin(now_se2(2));
	now_se2(2) += wz*time_resolution;
	now_se2(2) = normyaw(now_se2(2));
	
	moma_state.chassis_odom.header.stamp = ros::Time::now();
	moma_state.chassis_odom.pose.pose.position.x  = now_se2.x();
	moma_state.chassis_odom.pose.pose.position.y  = now_se2.y();
	Eigen::Quaterniond q = euler2rotation(M_PI_2, now_se2(2), 0.0);
	moma_state.chassis_odom.pose.pose.orientation.w  = cos(now_se2(2)/2);
	moma_state.chassis_odom.pose.pose.orientation.z  = sin(now_se2(2)/2);
	moma_state.chassis_odom.twist.twist.linear.x  = vx;
	moma_state.chassis_odom.twist.twist.angular.z = wz;
	moma_marker.markers[0].header.stamp = ros::Time::now();
	moma_marker.markers[0].pose = moma_state.chassis_odom.pose.pose;
	moma_marker.markers[0].pose.position.z = moma_param.chassis_height;
	moma_marker.markers[0].pose.orientation.w = q.w();
	moma_marker.markers[0].pose.orientation.x = q.x();
	moma_marker.markers[0].pose.orientation.y = q.y();
	moma_marker.markers[0].pose.orientation.z = q.z();

	// lidar
	Eigen::Quaterniond chasq(cos(now_se2(2)/2), 0.0, 0.0, sin(now_se2(2)/2));
	Eigen::Vector3d chasv(now_se2.x(), now_se2.y(), moma_param.chassis_height);
	Eigen::Vector3d lidar_pos = chasq.matrix() * lidar_vec + chasv;
	lidar_odom = moma_state.chassis_odom;
	lidar_odom.pose.pose.position.x = lidar_pos(0);
	lidar_odom.pose.pose.position.y = lidar_pos(1);
	lidar_odom.pose.pose.position.z = lidar_pos(2);

	// arm
	// now_q = moma_cmd.q.data;
	for (size_t i=0; i<moma_param.dof_num; i++)
		now_q[i] = moma_cmd.q.data[i];
		// now_q[i] += moma_cmd.dq.data[i]*time_resolution;
	
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
		moma_state.arm_odom[i].pose.pose = moma_marker.markers[2+i].pose;
		moma_state.arm_odom[i].twist.twist.linear.x = now_q[i];
		moma_state.arm_odom[i].twist.twist.angular.z = moma_cmd.dq.data[i];
	}

	// gripper
	moma_marker.markers.back().pose = moma_marker.markers[8].pose;
	if (moma_cmd.gripper_state)
	{
		moma_marker.markers.back().color.r = 0.0;
		moma_marker.markers.back().color.g = 0.0;
		moma_marker.markers.back().color.b = 0.0;
	}
	else
	{
		moma_marker.markers.back().color.g = 1.0;
	}

	// collision detection
	Eigen::VectorXd moma_pos(3+moma_param.dof_num);
	moma_pos.head(3) = now_se2;
	for (size_t i=0; i<moma_param.dof_num; i++)
		moma_pos[3+i] = now_q[i];
	
	Eigen::VectorXi collision_link;
	moma_param.isSelfCollision(moma_pos, collision_link);

	visualization_msgs::MarkerArray cylinder_msg = moma_param.getColliCylinderArray(moma_pos);
	for (size_t i=0; i<cylinder_msg.markers.size(); i++)
	{
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
		Eigen::Matrix3d cyR = Eigen::Quaterniond(cylinder_msg.markers[i].pose.orientation.w,
												cylinder_msg.markers[i].pose.orientation.x,
												cylinder_msg.markers[i].pose.orientation.y,
												cylinder_msg.markers[i].pose.orientation.z)
												.toRotationMatrix().transpose();
		pcl::PointXYZ pp;
		pp.x = cylinder_msg.markers[i].pose.position.x;
		pp.y = cylinder_msg.markers[i].pose.position.y;
		pp.z = cylinder_msg.markers[i].pose.position.z;
		double scale_x = cylinder_msg.markers[i].scale.x / 2.0;
		double scale_z = cylinder_msg.markers[i].scale.z / 2.0;
		double radius = sqrt(scale_x*scale_x + scale_z*scale_z) * 1.01;
		bool collision = false;
		if (has_map && kd_tree.radiusSearch(pp, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
		{
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) 
			{
				pcl::PointXYZ pt = cloud_map.points[pointIdxRadiusSearch[j]];
				Eigen::Vector3d pt_cy = Eigen::Vector3d(pt.x, pt.y, pt.z);
				pt_cy = cyR * (pt_cy - Eigen::Vector3d(pp.x, pp.y, pp.z));
				if (pt_cy[2] > -scale_z && pt_cy[2] < scale_z 
					&& pt_cy.head(2).norm() < scale_x)
				{
					collision = true;
					break;
				}
			}
		}

		moma_marker.markers[i].color.r = 0.5;
		moma_marker.markers[i].color.g = 0.5;
		moma_marker.markers[i].color.b = 0.5;
		// if (!collision && collision_link[i] == 0)
		// {
		// 	moma_marker.markers[i].color.r = 0.5;
		// 	moma_marker.markers[i].color.g = 0.5;
		// 	moma_marker.markers[i].color.b = 0.5;
		// }
		// else
		// {
		// 	if (collision)
		// 		moma_marker.markers[i].color.r = 1.0;
		// 	if (collision_link[i] != 0)
		// 		moma_marker.markers[i].color.b = 1.0;
		// }
	}
	state_pub.publish(moma_state);
	lidar_odom_pub.publish(lidar_odom);
	marker_pub.publish(moma_marker);
	cylinder_pub.publish(cylinder_msg);
	auto cma = moma_param.getColliMarkerArray(moma_pos);
	for (size_t i=0; i<cma.markers.size(); i++)
		cma.markers[i].header.stamp = ros::Time::now();
	sphere_pub.publish(cma);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_moma_node");
    ros::NodeHandle nh("~");

	initParams(nh);
	cmd_sub = nh.subscribe( "command", 1, rcvVelCmdCallBack);
	map_sub = nh.subscribe( "global_map", 1, rcvMapCallBack);
    state_pub  = nh.advertise<fake_moma::MomaState>("state", 1);
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
	cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinder", 1);
	sphere_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere", 1);
	lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/Odometry", 1);
	ros::Timer odom_timer = nh.createTimer(ros::Duration(time_resolution), simCallBack);

	ros::spin();
    return 0;
}