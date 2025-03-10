#ifndef __WAYPOINT_GENERATOR_H__
#define __WAYPOINT_GENERATOR_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <string>

class WaypointGenerator {
public:
    WaypointGenerator(){}
    ~WaypointGenerator(){}

    void init(ros::NodeHandle &nh);
private:
    ros::NodeHandle _nh;

    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;

    ros::Publisher _waypoints_pub;
    ros::Publisher _waypoints_visual_pub;

    ros::Time _triggered_time;

    std::string _waypoint_type;
    nav_msgs::Path _waypoints;

    nav_msgs::Odometry _odom;

    bool _is_visualized;
    bool _is_odom_ready;

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void publishVisualWaypoints();
};

#endif