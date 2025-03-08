#ifndef __ODOM_VISUALIZATION_H__
#define __ODOM_VISUALIZATION_H__

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "visualization_msgs/Marker.h"

class OdomVisualization {
public:
    OdomVisualization(ros::NodeHandle &nh);
    ~OdomVisualization();

private:
    ros::NodeHandle _nh;

    ros::Publisher _pose_pub;
    ros::Publisher _velocity_pub;
    ros::Publisher _path_pub;
    ros::Publisher _traj_pub;
    ros::Publisher _mesh_model_pub;

    ros::Subscriber _odom_sub;
    ros::Subscriber _cmd_sub;

    geometry_msgs::PoseStamped _pose;
    visualization_msgs::Marker _vel;
    nav_msgs::Path _path;
    visualization_msgs::Marker _traj;
    visualization_msgs::Marker _mesh_model;

    std::string _mesh_resource;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr cmd);
};

#endif