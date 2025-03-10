#include "plan_manage/astar_replan_fsm.h"

void AStarFSM::init(ros::NodeHandle &nh)
{
    _nh = nh;

    _exec_timer = nh.createTimer(ros::Duration(0.1),&AStarFSM::execCallback,this);
    
    _odom_sub = nh.subscribe("odom",10,&AStarFSM::odomCallback,this);
    _waypoints_sub = nh.subscribe("waypoints",10,&AStarFSM::waypointsCallback,this);
}

void AStarFSM::odomCallback(nav_msgs::Odometry::ConstPtr &msg)
{

}

void AStarFSM::execCallback(ros::TimerEvent &e)
{
    static int fsm_num = 0;
    fsm_num += 1;

    switch (_fsm_state){
        case FSM_EXEC_STATE::INIT:
            if(!_trigger)
                return;
            if(!_have_odom)
                return;

            changeFSMState(FSM_EXEC_STATE::WAIT_TARGET,"AStarFSM");
            break;
        case FSM_EXEC_STATE::WAIT_TARGET:
            if(!_have_target)
                return;
            changeFSMState(FSM_EXEC_STATE::GEN_NEW_TRAJ,"AStarFSM");
            break;
        case FSM_EXEC_STATE::GEN_NEW_TRAJ:
            _start_pos = _odom_pos;
            _start_vel = _odom_vel;
            _start_acc.setZero();



    default:
        break;
    }
}

bool AStarFSM::callAStarReplan(void)
{
    
}