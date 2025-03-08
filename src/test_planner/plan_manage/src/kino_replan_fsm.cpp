#include "plan_manage/kino_replan_fsm.h"

void KinoReplanFSM::init(ros::NodeHandle &nh)
{
    _nh = nh;

    _odom_sub = nh.subscribe("/odom_world",1,KinoReplanFSM::odomCallback,this);
    _exec_timer = nh.createTimer(ros::Duration(0.1),KinoReplanFSM::execCallback,this);
}

void KinoReplanFSM::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{

}

void KinoReplanFSM::execCallback(const ros::TimerEvent& e)
{
    static int fsm_num = 0;
    fsm_num++;

    switch (_exec_state){
    case FSC_EXEC_STATE::INIT:
        if(!_have_odom)
            return;
        if(!_trigger)
            return;
        changeFSMExecState(FSC_EXEC_STATE::WAIT_TARGET,"FSM");
        break;
    case FSC_EXEC_STATE::WAIT_TARGET:
        if(!_have_target)
            return;
        changeFSMExecState(FSC_EXEC_STATE::GEN_NEW_TRAJ,"FSM");
        break;
    case FSC_EXEC_STATE::GEN_NEW_TRAJ:
        bool success = callKinodynamicReplan();
        if(success){
            changeFSMExecState(FSC_EXEC_STATE::EXEC_TRAJ,"FSM");
        } else{
            changeFSMExecState(FSC_EXEC_STATE::GEN_NEW_TRAJ,"FSM");
        }
        break;
    case FSC_EXEC_STATE::EXEC_TRAJ:
        break;
    default:
        break;
    }


}

bool KinoReplanFSM::callKinodynamicReplan(void)
{

}

void KinoReplanFSM::changeFSMExecState(KinoReplanFSM::FSC_EXEC_STATE new_state,std::string pos_call)
{
    std::string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
    int pre_state = int(_exec_state);
    _exec_state = new_state;
    std::cout << "[" + pos_call +"]: from" + state_str[pre_state] + "to" + state_str[int(new_state)] << std::endl;
}