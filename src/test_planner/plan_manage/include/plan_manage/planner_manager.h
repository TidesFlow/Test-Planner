#ifndef __PLANNER_MANAGER_H__
#define __PLANNER_MANAGER_H__

#include <Eigen/Dense>

#include <ros/ros.h>
#include <path_searching/astar.h>

class PlannerManager {
public:
    PlannerManager(){}
    ~PlannerManager(){}

    void init(ros::NodeHandle &nh);
private:
    AStar a_star_planner;

    bool aStarReplan(Eigen::Vector3d start_pos,Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pos,Eigen::Vector3d end_vel);
};

#endif