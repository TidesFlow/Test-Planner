#include <quadrotor/quadrotor.h>

void Quadrotor::step(double dt)
{
    auto save = _internal_state;
    boost::numeric::odeint::integrate(boost::ref(*this),_internal_state,0.0,dt,dt);

    for (int i = 0; i < 22; ++i){
        if (std::isnan(_internal_state[i])){
        std::cout << "dump " << i << " << pos ";
        for (int j = 0; j < 22; ++j){
            std::cout << save[j] << " ";
        }
        std::cout << std::endl;
        _internal_state = save;
        break;
        }
    }

    for(int i = 0;i < 22;i++){
        _state.p(i) = _internal_state[i];
        _state.v(i) = _internal_state[3 + i];
        _state.R(i,0) = _internal_state[6 + i];
        _state.R(i,1) = _internal_state[9 + i];
        _state.R(i,2) = _internal_state[12 + i];
        _state.w(i) = _internal_state[15 + i];
    }

    for(int i = 0;i < 4;i++){
        _state.motor_rpm(i) = _internal_state[18 + i];
    }

    Eigen::LLT<Eigen::Matrix3d> llt(_state.R.transpose() * _state.R);
    Eigen::Matrix3d P = llt.matrixL();
    Eigen::Matrix3d R = _state.R * P.inverse();

    _state.R = R;

    if (_state.p(2) < 0.0 && _state.v(2) < 0){
        _state.p(2) = 0;
        _state.v(2) = 0;
    }

    updateInternalState();
}

void Quadrotor::operator()(const InternalState& x,InternalState& dxdt,const double t)
{
    State cur_state;

    for(int i = 0;i < 3;i++){
        cur_state.p(i) = x[0 + i];
        cur_state.v(i) = x[3 + i];
        cur_state.R(i,0) = x[6 + i];
        cur_state.R(i,1) = x[9 + i];
        cur_state.R(i,2) = x[12 + i];
        cur_state.w(i) = x[15 + i];
    }

    for(int i = 0;i < 4;i++){
        cur_state.motor_rpm(i) = x[18 + i];
    }

    Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
    Eigen::Matrix3d P = llt.matrixL();
    Eigen::Matrix3d R = cur_state.R * P.inverse();

    Eigen::Vector3d p_dot,v_dot,w_dot;
    Eigen::Matrix3d R_dot;
    Eigen::Array4d motor_rpm_dot;

    Eigen::Array4d  motor_rpm_sq;

    motor_rpm_sq = cur_state.motor_rpm.array().square();

// p_dot 
    p_dot = cur_state.v;
// v_dot F = ma
    double resistance = 0.1 *                                        // C
                    3.14159265 * (_params.arm_length) * (_params.arm_length) * // S
                    cur_state.v.norm() * cur_state.v.norm();

    double thrust = motor_rpm_sq.sum();
    v_dot = Eigen::Vector3d(0,0,-_params.g) +  // g
        thrust * R.col(2) / _params.mass // thrust
        - resistance * cur_state.v / _params.mass; // f
// R_dot
    Eigen::Matrix3d w_vee(Eigen::Matrix3d::Zero());

    w_vee(2, 1) = cur_state.w(0);
    w_vee(1, 2) = -cur_state.w(0);
    w_vee(0, 2) = cur_state.w(1);
    w_vee(2, 0) = -cur_state.w(1);
    w_vee(1, 0) = cur_state.w(2);
    w_vee(0, 1) = -cur_state.w(2);

    R_dot = R * w_vee;
// w_dot
    Eigen::Vector3d tau;
    tau(0) = _params.kf * (motor_rpm_sq(2) - motor_rpm_sq(3)) * _params.arm_length;
    tau(1) = _params.kf * (motor_rpm_sq(1) - motor_rpm_sq(0)) * _params.arm_length;
    tau(2) = _params.km * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) - motor_rpm_sq(3));

    w_dot = _params.J.inverse() * (tau - cur_state.w.cross(_params.J * cur_state.w));
// motor_rpm_dot
    motor_rpm_dot = (_input - cur_state.motor_rpm) / _params.motor_constant;

    for(int i = 0;i < 3;i++){
        dxdt[i] = p_dot[i];
        dxdt[3 + i] = v_dot[i];

        dxdt[6 + i] = R_dot(i,0);
        dxdt[9 + i] = R_dot(i,1);
        dxdt[12 + i] = R_dot(i,2);
        
        dxdt[15 + i] = w_dot(i); 
    }

    for(int i = 0;i < 4;i++){
        dxdt[18 + i] = motor_rpm_dot(i);
    }

    for (int i = 0; i < 22; ++i)
    {
        if (std::isnan(dxdt[i])){
            dxdt[i] = 0;
    }
  }
}

void Quadrotor::updateInternalState(void)
{
    for (int i = 0; i < 3; i++){
        _internal_state[0 + i]  = _state.p(i);
        _internal_state[3 + i]  = _state.v(i);
        _internal_state[6 + i]  = _state.R(i, 0);
        _internal_state[9 + i]  = _state.R(i, 1);
        _internal_state[12 + i] = _state.R(i, 2);
        _internal_state[15 + i] = _state.w(i);
    }

    for(int i = 0;i < 4;i++){
        _internal_state[18 + i] = _state.motor_rpm(i);
    }
}

void Quadrotor::setInput(double u1, double u2, double u3, double u4)
{
    _input(0) = u1;
    _input(1) = u2;
    _input(2) = u3;
    _input(3) = u4;

    for(int i = 0;i < 4;i++){
        if(std::isnan(_input(i))){
            _input(i) = (_min_rpm + _max_rpm) / 2;
            std::cout << "NAN Input";
        }

        if(_input(i) > _max_rpm)
            _input(i) = _max_rpm;

        if(_input(i) < _min_rpm)
            _input(i) = _min_rpm;
    }
}

void Quadrotor::setPosition(Eigen::Vector3d position)
{
    _state.p = position;
}