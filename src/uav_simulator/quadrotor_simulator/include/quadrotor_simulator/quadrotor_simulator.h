#ifndef __QUADROTOR_SIMULATOR_H__
#define __QUADROTOR_SIMULATOR_H__

#include <ros/ros.h>
#include "quadrotor/quadrotor.h"
#include <quadrotor_msgs/SO3Command.h>

class QuadrotorSimulator{
public:
    struct Control {
        double rpm[4];
    };

    struct Command {
        float force[3];
        float qx, qy, qz, qw;
        float kR[3];
        float kOm[3];
        float corrections[3];
        float current_yaw;
        bool  use_external_yaw;
    };

    QuadrotorSimulator(ros::NodeHandle nh);

    void process(void);
private:
    Quadrotor _quadrotor;
    
    ros::Subscriber _cmd_sub;

    Control _control;
    Command _command;

    double _simulation_rate;
    double _dt;

    Control getControl(void);

    void cmdCallback(const quadrotor_msgs::SO3Command::ConstPtr& msg);
};

#endif