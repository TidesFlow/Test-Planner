#include "odom_visualization/odom_visualization.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "string.h"

OdomVisualization::OdomVisualization(ros::NodeHandle &nh)
{
    _nh = nh;

    _cmd_sub = nh.subscribe("cmd",100,&OdomVisualization::cmdCallback,this,ros::TransportHints().tcpNoDelay());
    _odom_sub = nh.subscribe("odom",100,&OdomVisualization::odomCallback,this,ros::TransportHints().tcpNoDelay());

    _pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose",100,true);
    _velocity_pub = nh.advertise<visualization_msgs::Marker>("velocity",100,true);
}

void OdomVisualization::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(msg->header.frame_id == std::string("null")){
        return;
    }

    Eigen::Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    Eigen::Vector3d euler_angles = q.toRotationMatrix().eulerAngles(2,1,0);

    Eigen::VectorXd pose(6);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;
    pose(3) = euler_angles(0);
    pose(4) = euler_angles(1);
    pose(5) = euler_angles(2);

    Eigen::Vector3d vel;
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    _pose.header = msg->header;
    _pose.header.stamp = msg->header.stamp;
    _pose.header.frame_id = std::string("world");

    _pose.pose.position.x = pose(0);
    _pose.pose.position.y = pose(1);
    _pose.pose.position.z = pose(2);

    Eigen::MatrixX3d rotation_maxtirx;
    rotation_maxtirx = Eigen::AngleAxisd(pose(4), Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pose(5), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(pose(6), Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q(rotation_maxtirx);

    _pose.pose.orientation.w = q.w();
    _pose.pose.orientation.x = q.x();
    _pose.pose.orientation.y = q.y();
    _pose.pose.orientation.z = q.z();

    _pose_pub.publish(_pose);

    Eigen::Vector3d vel_euler_angles;
    vel_euler_angles(0) = atan2(vel(1),vel(0));
    vel_euler_angles(1) = -atan2(vel(2),vel_euler_angles.segment(0,1).norm());
    vel_euler_angles(2) = 0;
    Eigen::MatrixX3d vel_rotation_maxtirx;
    vel_rotation_maxtirx = Eigen::AngleAxisd(vel_euler_angles(0), Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(vel_euler_angles(1), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(vel_euler_angles(2), Eigen::Vector3d::UnitX());

    Eigen::Quaterniond vel_q(vel_rotation_maxtirx);

    _vel.header.frame_id = std::string("world");
    _vel.header.stamp = msg->header.stamp;
    _vel.ns = std::string("velocity");
    _vel.id = 0;
    _vel.type = visualization_msgs::Marker::ARROW;
    _vel.pose.position.x = pose(0);
    _vel.pose.position.y = pose(1);
    _vel.pose.position.z = pose(2);
    _vel.pose.orientation.w = vel_q.w();
    _vel.pose.orientation.x = vel_q.x();
    _vel.pose.orientation.y = vel_q.y();
    _vel.pose.orientation.z = vel_q.z();
    _vel.scale.x = 0.05;
    _vel.scale.x = 0.05;
    _vel.scale.x = 0.05;
    _vel.color.a = 1.0;
    _vel.color.r = 1.0;
    _vel.color.g = 1.0;
    _vel.color.b = 1.0;
    _velocity_pub.publish(_vel);

    static ros::Time prev_path_t = msg->header.stamp;
    if((msg->header.stamp - prev_path_t).toSec() > 0.1) {
        prev_path_t = msg->header.stamp;
        _path.header = _pose.header;
        _path.poses.push_back(_pose);
        _path_pub.publish(_path);
    }

    static Eigen::VectorXd prev_pose(pose);
    static ros::Time prev_pose_t = msg->header.stamp; 
    if((msg->header.stamp - prev_pose_t).toSec() > 0.5) {
        _traj.header.frame_id = std::string("world");
        _traj.header.stamp = ros::Time::now();
        _traj.ns = std::string("trajectory");
        _traj.type = visualization_msgs::Marker::LINE_LIST;
        _traj.action = visualization_msgs::Marker::ADD;
        _traj.pose.position.x = 0;
        _traj.pose.position.y = 0;
        _traj.pose.position.z = 0;
        _traj.pose.orientation.w = 0;
        _traj.pose.orientation.x = 0;
        _traj.pose.orientation.y = 0;
        _traj.pose.orientation.z = 0;
        _traj.scale.x = 0.1;
        _traj.scale.y = 0;
        _traj.scale.z = 0;
        _traj.color.r = 0.0;
        _traj.color.g = 1.0;
        _traj.color.b = 0.0;
        _traj.color.a = 0.8;

        geometry_msgs::Point p;
        p.x = prev_pose(0);
        p.y = prev_pose(1);
        p.z = prev_pose(2);
        _traj.points.push_back(p);
        p.x = pose(0);
        p.y = pose(1);
        p.z = pose(2);
        _traj.points.push_back(p);

        std_msgs::ColorRGBA color;
        color.r = 1;
        color.g = 1;
        color.b = 1;
        color.a = 1;
        _traj.colors.push_back(color);
        _traj.colors.push_back(color);
        prev_pose_t = msg->header.stamp;
        prev_pose = pose;

        _traj_pub.publish(_traj);
    }

    _mesh_model.header.frame_id = std::string("world");
    _mesh_model.header.stamp = msg->header.stamp;
    _mesh_model.ns = "mesh";
    _mesh_model.id = 0;
    _mesh_model.type = visualization_msgs::Marker::MESH_RESOURCE;
    _mesh_model.action = visualization_msgs::Marker::ADD;
    _mesh_model.pose.position.x = msg->pose.pose.position.x;
    _mesh_model.pose.position.y = msg->pose.pose.position.y;
    _mesh_model.pose.position.z = msg->pose.pose.position.z;
    _mesh_model.pose.orientation.w = msg->pose.pose.orientation.w;
    _mesh_model.pose.orientation.x = msg->pose.pose.orientation.x;
    _mesh_model.pose.orientation.y = msg->pose.pose.orientation.y;
    _mesh_model.pose.orientation.z = msg->pose.pose.orientation.z;
    _mesh_model.scale.x = 0.05;
    _mesh_model.scale.x = 0.05;
    _mesh_model.scale.x = 0.05;
    _mesh_model.color.a = 1.0;
    _mesh_model.color.r = 1.0;
    _mesh_model.color.g = 1.0;
    _mesh_model.color.b = 1.0;
    _mesh_model.mesh_resource = _mesh_resource;
    _mesh_model_pub.publish(_mesh_model);
}


void OdomVisualization::cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr cmd)
{

}