#include "waypoint_generator/waypoint_generator.h"

void WaypointGenerator::init(ros::NodeHandle &nh)
{
    _nh = nh;

    _goal_sub = nh.subscribe("goal",10,&WaypointGenerator::goalCallback,this);


    _waypoints_pub = nh.advertise<nav_msgs::Path>("waypoints",50);
    _waypoints_visual_pub = nh.advertise<geometry_msgs::PoseArray>("waypoints_visual",50);
}

void WaypointGenerator::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(_waypoint_type == std::string("manual-lonely-waypoint")){
        if(msg->pose.position.z > -0.1){
            geometry_msgs::PoseStamped pose = *msg;
            _waypoints.poses.clear();
            _waypoints.poses.push_back(pose);

            if(_is_visualized)
                publishVisualWaypoints();

            _waypoints.header.frame_id = std::string("world");
            _waypoints.header.stamp = ros::Time::now();
            _waypoints_pub.publish(_waypoints);

            _waypoints.poses.clear();
        }
    }
}

void WaypointGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    _odom = *msg;
}

void WaypointGenerator::publishVisualWaypoints()
{
    nav_msgs::Path waypoints_visual = _waypoints;
    geometry_msgs::PoseArray pose_array;

    pose_array.header.frame_id = std::string("world");
    pose_array.header.stamp = ros::Time::now();

    {
        geometry_msgs::Pose init_pose;
        
        pose_array.poses.push_back(init_pose);
    }

    for(auto iter = waypoints_visual.poses.begin();iter != waypoints_visual.poses.end();it++) {
        geometry_msgs::Pose pose;
        pose = iter->pose;
        pose_array.poses.push_back(pose);
    }

    _waypoints_visual_pub.publish(pose_array);
    
}