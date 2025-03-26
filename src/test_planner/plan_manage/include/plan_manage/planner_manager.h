#ifndef __PLANNER_MANAGER_H__
#define __PLANNER_MANAGER_H__

#include <Eigen/Dense>

#include <ros/ros.h>

#include <path_searching/astar.h>
#include <path_searching/kinodynamic_astar.h>

#include <plan_env/edt_environment.h>

class PlannerManager {
public:
    PlannerManager(){}
    ~PlannerManager(){}

    void init(ros::NodeHandle &nh);
private:

};

#endif