#include "bspline/non_uniform_bspline.h"

NonUniformBSpline::NonUniformBSpline(const Eigen::MatrixXd &control_points,const int &order,const double &interval)
{
    setNonUniformBSpline(control_points,order,interval);
}

void NonUniformBSpline::setNonUniformBSpline(const Eigen::MatrixXd &control_points,const int &order,const double &interval)
{
    _control_points = control_points;
    _interval = interval;

    _p = order;
    _n = _control_points.rows() - 1;
    _m = _p + _n + 1;

    _u = Eigen::VectorXd(_m + 1);

    for(int i = 0;i <= _m;i++){
        if(i < _p){
            _u(i) = double(-_p + i) * _interval;
        } else if(i >= _p && i <=  _m - _p) {
            _u(i) = _u(i - 1) + _interval;
        } else {
            _u(i) = _u(i - 1) + _interval;
        }
    }
}
/* reference:https://en.wikipedia.org/wiki/De_Boor%27s_algorithm*/
Eigen::VectorXd NonUniformBSpline::evaluateDedoor(const double &u)
{
    double ub = std::min(std::max(u,_u(_p)),_u(_m - _p));

    int k = _p;
    while (true){
        if(_u(k + 1) >= ub) 
            break;
        k += 1;
    }

    std::vector<Eigen::VectorXd> d;
    for(int i = 0; i < _p;i++){
        d.push_back(_control_points.row(i - _p + k));
    }

    for(int r = 1;r <= _p;r++){
        for(int j = _p;j > r - 1;j--){
            double alpha = (ub - _u[j + k - _p]) / (_u[j + 1 + k - r] - _u[j + k - _p]);
            d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j]; 
        }
    }

    return d[_p];
}

Eigen::MatrixXd NonUniformBSpline::getDerivativePoints(void)
{
    Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(_control_points.rows() - 1, _control_points.cols());
    for(int i = 0;i < _n - 1;i++){
        ctp.row(i) = _p / (_u(i + _p + 1) - _u(i + 1))* (_control_points.row(i + 1) - _control_points.row(i));
    }

    return ctp;
}

NonUniformBSpline NonUniformBSpline::getDerivative(void)
{
    Eigen::MatrixXd new_points = getDerivativePoints();
    NonUniformBSpline derivative(new_points,_p - 1,_interval);

    Eigen::VectorXd new_knot(_u.rows() - 2);
    new_knot = _u.segment(1,_u.rows() - 2);
    derivative.setKnot(new_knot);
    
    return derivative;
}

void NonUniformBSpline::setKnot(const Eigen::VectorXd &knot)
{
    _u = knot;
}

Eigen::VectorXd NonUniformBSpline::getKnot(void)
{
    return _u;
}

void NonUniformBSpline::getTimeSpan(double &up,double &um_p)
{
    up = _u(_p);
    um_p = _u(_m-_p);
}

Eigen::VectorXd NonUniformBSpline::getControlPoints(void)
{
    return _control_points;
}

double NonUniformBSpline::getInterval(void)
{   
    return _interval;
}

void NonUniformBSpline::setPhysicalLimits(const double& vel,const double& acc)
{
    _limit_vel = vel;
    _limit_acc = acc;
    _limit_ratio_ = 1.1;
}

bool NonUniformBSpline::checkFeasilbility(bool show)
{
    bool is_feasible = true;

    // check velocity feasible
    Eigen::MatrixXd P = _control_points;
    int dimension = _control_points.cols();

    for(int i = 0; i < P.rows() - 1;i++){
        Eigen::VectorXd vel = _p / (_u(i + _p + 1) - _u(i + 1))* (_control_points.row(i + 1) - _control_points.row(i));
        
        if (fabs(vel(0)) > _limit_vel + 1e-4 || fabs(vel(1)) > _limit_vel + 1e-4 ||
            fabs(vel(2)) > _limit_vel + 1e-4) {
            if (show) 
                std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;
            
            is_feasible = false;
        }
    }

    for(int i = 0; i < P.rows() - 2;i++){
        Eigen::VectorXd acc = (_p - 1) / (_u(i + _p + 1) - _u(i + 2)) * _p *
                            ((P.row(i + 2) - P.row(i + 1)) / (_u(i + _p + 2) - _u(i + 2)) -
                            (P.row(i + 1) - P.row(i)) / (_u(i + _p + 1) - _u(i + 1)));
        if (fabs(acc(0)) > _limit_acc + 1e-4 || fabs(acc(1)) > _limit_acc + 1e-4 ||
            fabs(acc(2)) > _limit_acc + 1e-4) {
            if (show) 
                std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;
            
            is_feasible = false;
        }
    }

    return is_feasible;
}

double NonUniformBSpline::checkRatio()
{
    Eigen::MatrixXd P = _control_points;
    int dimension = _control_points.cols();

    double max_vel = -1.0;
    for(int i = 0; i < P.rows() - 1;i++){
        Eigen::VectorXd vel = _p / (_u(i + _p + 1) - _u(i + 1))* (_control_points.row(i + 1) - _control_points.row(i));
        for(int j = 0;j < dimension;j++){
            max_vel = std::max(max_vel,vel(j));
        }
    }

    double max_acc = -1.0;
    for(int i = 0; i < P.rows() - 1;i++){
        Eigen::VectorXd acc = (_p - 1) / (_u(i + _p + 1) - _u(i + 2)) * _p *
                            ((P.row(i + 2) - P.row(i + 1)) / (_u(i + _p + 2) - _u(i + 2)) -
                            (P.row(i + 1) - P.row(i)) / (_u(i + _p + 1) - _u(i + 1)));
        for(int j = 0;j < dimension;j++){
            max_acc = std::max(max_acc,acc(j));
        }
    }

    double ratio = std::max(max_vel / _limit_vel,sqrt(fabs(max_acc) / _limit_acc));
    return ratio;
}

bool NonUniformBSpline::reallocateTime(bool show)
{
    bool is_feasible = true;

    Eigen::MatrixXd P = _control_points;
    int dimension = _control_points.cols();

    for(int i = 0; i < P.rows() - 1;i++){
        Eigen::VectorXd vel = _p / (_u(i + _p + 1) - _u(i + 1))* (_control_points.row(i + 1) - _control_points.row(i));
        if (fabs(vel(0)) > _limit_vel + 1e-4 || fabs(vel(1)) > _limit_vel + 1e-4 ||
            fabs(vel(2)) > _limit_vel + 1e-4) {
            if (show) 
                std::cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << std::endl;
            
            is_feasible = false;
            double max_vel = -1.0;
            for(int j = 0;j < dimension;j++){
                max_vel = std::max(max_vel,vel(j));
            }

            double radio = std::min(max_vel / _limit_vel + 1e-4,_limit_ratio_);
            
            double time_original = _u(i + _p + 1) - _u(i + 1);
            double time_new = time_original * radio;
            double delta_t = time_new - time_original;
            double t_inc = delta_t / _p;

            for(int j = i + 2;j <= i + _p + 1;j++){
                _u(j) += double(j - i - 1) * t_inc;
            }

            for(int j = i + _p + 2;j < _u.rows();j++){
                _u(j) += delta_t;
            }
        }
    }


    for(int i = 0; i < P.rows() - 1;i++){
        Eigen::VectorXd acc = (_p - 1) / (_u(i + _p + 1) - _u(i + 2)) * _p *
                            ((P.row(i + 2) - P.row(i + 1)) / (_u(i + _p + 2) - _u(i + 2)) -
                            (P.row(i + 1) - P.row(i)) / (_u(i + _p + 1) - _u(i + 1)));
        if (fabs(acc(0)) > _limit_acc + 1e-4 || fabs(acc(1)) > _limit_acc + 1e-4 ||
            fabs(acc(2)) > _limit_acc + 1e-4) {
            if (show) 
                std::cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << std::endl;
            
            is_feasible = false;
            double max_acc = -1.0;
            for(int j = 0;j < dimension;j++){
                max_acc = std::max(max_acc,acc(j));
            }

            double radio = std::min(std::sqrt(max_acc / _limit_acc) + 1e-4,_limit_ratio_);
            
            double time_original = _u(i + _p + 1) - _u(i + 2);
            double time_new = time_original * radio;
            double delta_t = time_new - time_original;
            double t_inc = delta_t / double(_p - 1);


            if (i == 1 || i == 2) {
                for (int j = 2; j <= 5; ++j) {
                    _u(j) += double(j - 1) * t_inc;
                }

                for (int j = 6; j < _u.rows(); ++j) {
                    _u(j) += 4.0 * t_inc;
                }
            } else{
                for(int j = i + 3;j <= i + _p + 1;j++){
                    _u(j) += double(j - i - 2) * t_inc;
                }

                for(int j = i + _p + 2;j < _u.rows();j++){
                    _u(j) += delta_t;
                }
            }
        }
    }

    return is_feasible;
}

void NonUniformBSpline::parameterizeToBspline(std::vector<Eigen::Vector3d> points_set,
                            std::vector<Eigen::Vector3d> start_end_derivatives,
                            double ts,
                            Eigen::MatrixXd &control_points)
{
    int K = points_set.size();

    // default degree 3
    int p = 3;
    // m = K + 2 * p -1
    int m = K + 5;
    // n = m - p -1
    int n = K + 1;
    
    Eigen::Vector3d prow,vrow,arow;

    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4,K + 2);

    for(int i = 0;i < K;i++){
        A.block(i,i,1,3) = prow;
    }

    A.block(K,0,1,3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1,K - 1,1,3) = (1 / ts / ts) * arow;
    A.block(K + 2,0,1,3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 3,K - 1,1,3) = (1 / ts / ts) * arow;

    Eigen::VectorXd bx(K + 4),by(K + 4),bz(K + 4);

    for(int i = 0;i < K;i++){
        bx(i) = points_set[i](0);
        by(i) = points_set[i](1);
        bz(i) = points_set[i](2);
    }

    for(int i = 0;i < 4;i++){
        bx(K + i) = start_end_derivatives[i](0);
        by(K + i) = start_end_derivatives[i](1);
        bz(K + i) = start_end_derivatives[i](2);
    }

    Eigen::VectorXd control_points_x = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd control_points_y = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd control_points_z = A.colPivHouseholderQr().solve(bz);

    control_points.resize(K + 2,3);
    control_points.col(0) = control_points_x;
    control_points.col(1) = control_points_y;
    control_points.col(2) = control_points_z;
}