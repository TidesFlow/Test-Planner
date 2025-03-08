#ifndef __QUADROTOR_H__
#define __QUADROTOR_H__

#include <Eigen/Dense>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

class Quadrotor{
public:
    struct State {
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Matrix3d R;
        Eigen::Vector3d w;
        Eigen::Array4d motor_rpm;
    };

    struct Params
    {
        double g;
        double mass;
        double arm_length;
        Eigen::Matrix3d J;

        double kf;
        double km;

        double motor_constant;
    };

    Quadrotor();
    ~Quadrotor();

    void setPosition(Eigen::Vector3d position);
    void setInput(double u1, double u2, double u3, double u4);

    void step(double dt);
    void updateInternalState(void);

    typedef boost::array<double,22> InternalState;
    void operator()(const InternalState& x,InternalState& dxdt,const double t);

    
private:
    State _state;
    InternalState _internal_state;

    Params _params;
    
    Eigen::Array4d _input;
    double _max_rpm,_min_rpm;
};

#endif