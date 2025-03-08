#ifndef __KINO_REPLAN_FSM_H__
#define __KINO_REPLAN_FSM_H__

#include <string.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class KinoReplanFSM {
public:
    enum FSC_EXEC_STATE{
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        REPLAN_NEW
    };
    
    KinoReplanFSM(){}
    ~KinoReplanFSM(){}

    void init(ros::NodeHandle &nh);
private:
    bool _trigger,_have_odom,_have_target;

    FSC_EXEC_STATE _exec_state;

    ros::NodeHandle _nh;

    ros::Timer _exec_timer;

    ros::Subscriber _odom_sub;
    ros::Subscriber _waypoint_sub;

    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void execCallback(const ros::TimerEvent& e);

    bool callKinodynamicReplan(void);

    void changeFSMExecState(KinoReplanFSM::FSC_EXEC_STATE new_state,std::string pos_call);
};


#endif