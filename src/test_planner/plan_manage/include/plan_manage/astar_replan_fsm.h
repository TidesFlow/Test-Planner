#ifndef __ASTAR_FSM_H__
#define __ASTAR_FSM_H__

#include <string>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "plan_manage/planner_manager.h"

class AStarFSM {
public:
    enum FSM_EXEC_STATE{
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        REPLAN_NEW
    };
    
    AStarFSM(){};
    ~AStarFSM(){};

    void init(ros::NodeHandle &nh);
private:
    PlannerManager planner_manager;

    bool _trigger,_have_odom,_have_target;
    FSM_EXEC_STATE _fsm_state;

    ros::NodeHandle _nh;

    ros::Subscriber _odom_sub;
    ros::Subscriber _waypoints_sub;

    Eigen::Vector3d _odom_pos,_odom_vel;
    Eigen::Vector3d _start_pos,_start_vel,_start_acc,_end_pos,_end_vel;

    ros::Timer _exec_timer;

    bool callAStarReplan(void);

    void changeFSMState(FSM_EXEC_STATE state,std::string pos_call);

    void odomCallback(nav_msgs::Odometry::ConstPtr &msg);
    void waypointsCallback(nav_msgs::Path::ConstPtr &msg);
    void execCallback(ros::TimerEvent &e);

};

#endif