#include <ros/ros.h>

#include "quadrotor/quadrotor.h"


int main(int argc,char *argv[])
{
    ros::init(argc,argv,"quadrotor_simulate_node");
    ros::NodeHandle nh("~");

    return 0;
}