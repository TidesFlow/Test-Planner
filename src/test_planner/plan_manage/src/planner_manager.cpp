#include "plan_manage/planner_manager.h"

void PlannerManager::init(ros::NodeHandle &nh)
{

}

bool PlannerManager::aStarReplan(Eigen::Vector3d start_pos,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pos,Eigen::Vector3d end_vel)   
{
    a_star_planner.reset();
    
    int status = a_star_planner.search(start_pos,end_pos);
    if(status == AStar::PLAN_RESULT::NO_PATH){
        a_star_planner.reset();
        status = a_star_planner.search(start_pos,end_pos);

        if(status == AStar::PLAN_RESULT::NO_PATH){
            return false;
        }
    }
}

