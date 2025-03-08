#include "quadrotor_simulator/quadrotor_simulator.h"

QuadrotorSimulator::QuadrotorSimulator(ros::NodeHandle nh)
{
    _cmd_sub = nh.subscribe<quadrotor_msgs::SO3Command>("cmd", 100, &QuadrotorSimulator::cmdCallback,this,
                                ros::TransportHints().tcpNoDelay());
    
    nh.param("rate/simulation", _simulation_rate, 1000.0);
    _dt = 1/_simulation_rate;

    double init_x,init_y,init_z;

    nh.param("simulator/init_state_x", init_x, 0.0);
    nh.param("simulator/init_state_y", init_y, 0.0);
    nh.param("simulator/init_state_z", init_z, 0.0);

    Eigen::Vector3d position = Eigen::Vector3d(init_x,init_y,init_z);
    _quadrotor.setPosition(position); 

}

QuadrotorSimulator::Control QuadrotorSimulator::getControl(void)
{

}

void QuadrotorSimulator::process(void)
{
    auto last = _control;
    _control = getControl();

    for(int i = 0;i < 4;i++){
        if(std::isnan(_control.rpm[i]))
            _control.rpm[i] = last.rpm[i];
    }

    _quadrotor.setInput(_control.rpm[0],_control.rpm[1],_control.rpm[2],_control.rpm[3]);
    _quadrotor.step(_dt);

}

void QuadrotorSimulator::cmdCallback(const quadrotor_msgs::SO3Command::ConstPtr& msg)
{
  _command.force[0]         = msg->force.x;
  _command.force[1]         = msg->force.y;
  _command.force[2]         = msg->force.z;
  _command.qx               = msg->orientation.x;
  _command.qy               = msg->orientation.y;
  _command.qz               = msg->orientation.z;
  _command.qw               = msg->orientation.w;
  _command.kR[0]            = msg->kR[0];
  _command.kR[1]            = msg->kR[1];
  _command.kR[2]            = msg->kR[2];
  _command.kOm[0]           = msg->kOm[0];
  _command.kOm[1]           = msg->kOm[1];
  _command.kOm[2]           = msg->kOm[2];
  _command.corrections[0]   = msg->aux.kf_correction;
  _command.corrections[1]   = msg->aux.angle_corrections[0];
  _command.corrections[2]   = msg->aux.angle_corrections[1];
  _command.current_yaw      = msg->aux.current_yaw;
  _command.use_external_yaw = msg->aux.use_external_yaw;
}


