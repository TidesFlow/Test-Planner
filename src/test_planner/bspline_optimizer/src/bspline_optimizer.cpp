#include "bspline_optimizer/bspline_optimizer.h"

void BsplineOptimizer::setEnvironment(const EDTEnvironment &env)
{

}
    
Eigen::MatrixXd BsplineOptimizer::bsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                        const int& cost_function, int max_num_id, int max_time_id)
{
    setControlPoints(points);
    setTimeInterval(ts);
    setCostFunction(cost_function);

    optimize();
    return _control_points;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points)
{
    _control_points = points;
}

void BsplineOptimizer::setTimeInterval(const double &ts)
{
    _interval_t = ts;
}
    
void BsplineOptimizer::setCostFunction(const int& cost_function)
{
    
}

void BsplineOptimizer::optimize(void)
{
    _iter_num = 0;
    _min_cost = std::numeric_limits<double>::max();

    const int point_num = _control_points.size();

    _g_q.resize(point_num);
    _g_smoothness.resize(point_num);
    _g_distance.resize(point_num);
    _g_feasiblity.resize(point_num);

    _variable_num = std::max(0,_dim * (point_num - 2 * _order));

    nlopt::opt opt(nlopt::algorithm(isQuadratic()?_algorithm1:_algorithm2),_variable_num);

    opt.set_min_objective(BsplineOptimizer::costFunction,this);
    opt.set_maxeval(_max_iteration_num[_max_num_id]);
    opt.set_maxtime(_max_iteration_time[_max_time_id]);
    opt.set_xtol_rel(1e-5);

    std::vector<double> q(_variable_num);
    for(int i = _order;i < point_num;i++){
        if( i >= point_num - _order) continue;
        for(int j = 0;j < _dim;j++){
            q[_dim * (i - _order) + j] = _control_points(i,j);
        }
    }

    if (_dim != 1) {
        std::vector<double> lb(_variable_num), ub(_variable_num);
        const double   bound = 10.0;
        for (int i = 0; i < _variable_num; ++i) {
            lb[i] = q[i] - bound;
            ub[i] = q[i] + bound;
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
    }

    try {
        double final_cost;
        nlopt::result result = opt.optimize(q,final_cost);
    } catch (std::exception& e){
        ROS_WARN("[Optimization]: nlopt exception");
    }

    for (int i = _order; i < _control_points.rows(); ++i) {
        if (i >= point_num - _order) continue;
        for (int j = 0; j < _dim; j++) {
            _control_points(i, j) = _best_variable[_dim * (i - _order) + j];
        }
    }
}

double BsplineOptimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data)
{
    BsplineOptimizer *bspline_opt = reinterpret_cast<BsplineOptimizer*>(func_data);
    double cost; 

    bspline_opt->combineCost(x,grad,cost);
    bspline_opt->_iter_num += 1;

    if(bspline_opt->_min_cost > cost){
        bspline_opt->_min_cost = cost;
        bspline_opt->_best_variable = x;
    }

    return cost;
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,double& f_combine)
{
    for(int i = 0;i < _order;i++){
        for(int j = 0;j < _dim;j++){
            _g_q[i][j] = _control_points(i,j);
        }
        for(int j = _dim;j < 3;j++){
            _g_q[i][j] = 0.0;
        }
    }

    for(int i = 0;i < _variable_num / _dim;i++){
        for(int j = 0;j < _dim;j++){
            _g_q[i + _order][j] = x[_dim * i + j];
        }
        for(int j = _dim;j < 3;j++){
            _g_q[i + _order][j] = 0.0;
        }
    }

    for (int i = 0; i < _order; i++) {
        for (int j = 0; j < _dim; ++j) {
            _g_q[_order + _variable_num / _dim + i][j] =
                _control_points(_control_points.rows() - _order + i, j);
        }
        for (int j = _dim; j < 3; ++j) {
            _g_q[_order + _variable_num / _dim + i][j] = 0.0;
        }
    }

    f_combine = 0.0;
    grad.resize(_variable_num);
    fill(grad.begin(), grad.end(), 0.0);

    /*  evaluate costs and their gradient  */
    double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
    f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

    if(_cost_function & SMOOTHNESS){
        calcSmoothnessCost(_g_q, f_smoothness, _g_smoothness);
        f_combine += _lambda1 * f_smoothness;
        for (int i = 0; i < _variable_num / _dim; i++)
            for (int j = 0; j < _dim; j++) 
                grad[_dim * i + j] += _lambda1 * _g_smoothness[i + _order](j);
    }

    if(_cost_function & DISTANCE){
        calcDistanceCost(_g_q,f_distance,_g_distance);
        f_combine += _lambda2 * f_distance;
        for (int i = 0; i < _variable_num / _dim; i++)
            for (int j = 0; j < _dim; j++) 
                grad[_dim * i + j] += _lambda2 * _g_distance[i + _order](j);
    }

    if(_cost_function & FEASIBILITY){
        calcDistanceCost(_g_q,f_feasibility,_g_feasiblity);
        f_combine += _lambda3 * f_distance;
        for (int i = 0; i < _variable_num / _dim; i++)
            for (int j = 0; j < _dim; j++) 
                grad[_dim * i + j] += _lambda3 * _g_feasiblity[i + _order](j);
    }
}

void BsplineOptimizer::calcFeasibilityCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient)
{
    cost = 0.0;
    
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    double ts,ts_inv2,vm2;

    ts = _interval_t;
    ts_inv2 = 1 / ts / ts;
    vm2 = _max_vel * _max_vel;

    /* --------------- velocity feasibility ---------- */
    for(int i = 0; i < q.size() - 1;i++){
        Eigen::Vector3d vi = q[i+1] - q[i];
        for(int j = 0;j < _dim;j++){
            double vd = vi(j) * vi(j) * ts_inv2 - vm2;
            if(vd > 0) {
                cost += std::pow(vd,2);

                double temp_v = 4.0 * vd * ts_inv2 * vi(j);
                gradient[i + 0](j) += - temp_v;
                gradient[i + 1](j) += - temp_v;
            }
        }
    }

    /* ---------------- accleration feasibility -------- */
    double ts_inv4,am2;
    ts_inv2 = 1 / ts / ts / ts / ts;
    am2 = _max_acc * _max_acc;

    for(int i = 0; i < q.size() - 2;i++){
        Eigen::Vector3d ai = q[i+2] - 2 * q[i+1] + q[i];
        for(int j = 0;j < _dim;j++){
            double ad = ai(j) * ai(j) * ts_inv4 - am2;
            if(ad > 0){
                cost += std::pow(ad,2);

                double temp_a = 4.0 * ad * ts_inv4 * ai(j);
                gradient[i + 0](j) += temp_a;
                gradient[i + 1](j) += -2 * temp_a;
                gradient[i + 1](j) += temp_a;
            }
        }
    }
}

void BsplineOptimizer::calcDistanceCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient)
{
    cost = 0.0;
    
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

    double distance;
    Eigen::Vector3d dist_grad;

    for(int i = _order;i < q.size() - _order;i++){
        _edt_env->evaluateEDTWithGrad(q[i],-1.0,distance,dist_grad);
        if(dist_grad.norm() > 1e-4) 
            dist_grad.normalize();

        if(distance < _distance_threshold){
            cost += std::pow(distance - _distance_threshold,2);
            gradient[i] += 2.0 * (distance - _distance_threshold) * dist_grad;
        }
    }
}

void BsplineOptimizer::calcSmoothnessCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient)
{
    cost = 0.0;
    
    Eigen::Vector3d zero(0, 0, 0);
    std::fill(gradient.begin(), gradient.end(), zero);

      Eigen::Vector3d jerk, temp_j;

    for (int i = 0; i < q.size() - _order; i++) {
        jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
        cost += jerk.squaredNorm();

        temp_j = 2.0 * jerk;

        gradient[i + 0] += -temp_j;
        gradient[i + 1] += 3.0 * temp_j;
        gradient[i + 2] += -3.0 * temp_j;
        gradient[i + 3] += temp_j;
    }
}