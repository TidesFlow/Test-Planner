#ifndef __BSPLINE_OPTIMIZER_H__
#define __BSPLINE_OPTIMIZER_H__

#include <vector>

#include <nlopt.hpp>

#include <ros/ros.h>

#include <Eigen/Dense>

#include <plan_env/edt_environment.h>

class BsplineOptimizer {
public:
    static const int SMOOTHNESS;
    static const int DISTANCE;
    static const int FEASIBILITY;
    static const int ENDPOINT;
    static const int GUIDE;
    static const int WAYPOINTS;

    static const int GUIDE_PHASE;
    static const int NORMAL_PHASE;

    BsplineOptimizer(){}
    ~BsplineOptimizer(){}

    void setEnvironment(const EDTEnvironment &env);
    
    Eigen::MatrixXd bsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                        const int& cost_function, int max_num_id, int max_time_id);

    void setControlPoints(const Eigen::MatrixXd& points);
    void setTimeInterval(const double &ts);
    void setCostFunction(const int& cost_function);

    void optimize(void);
private:
    EDTEnvironment *_edt_env;
    
    Eigen::MatrixXd _control_points;
    double _interval_t;
    int _order;
    int _dim;

    int _iter_num;
    double _min_cost;

    int _variable_num;
    std::vector<double> _best_variable;
    double _min_cost;

    int _algorithm1;
    int _algorithm2;
    int _max_iteration_num[4];
    double _max_iteration_time[4];
    int _max_num_id, _max_time_id;

    int _cost_function;
    /* ---------------- ---------------------*/
    double _lambda1;                // jerk smoothness weight
    double _lambda2;                // distance weight
    double _lambda3;                // feasibility weight
    double _lambda4;                // end point weight
    double _lambda5;                // guide cost weight
    double _lambda6;                // visibility cost weight
    double _lambda7;                // waypoints cost weight
    double _lambda8;                // acc smoothness
    /* -------------            -------------*/
    std::vector<Eigen::Vector3d> _g_q;
    std::vector<Eigen::Vector3d> _g_smoothness;
    std::vector<Eigen::Vector3d> _g_distance;
    std::vector<Eigen::Vector3d> _g_feasiblity;
    /* ------------ feasiability -------------*/
    double _max_vel,_max_acc;
    /* ------------ safe distance ------------*/
    double _distance_threshold;


    bool isQuadratic();
    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);


    void combineCost(const std::vector<double>& x, std::vector<double>& grad,double& f_combine);
    void calcFeasibilityCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient);
    void calcDistanceCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient);
    void calcSmoothnessCost(const std::vector<Eigen::Vector3d>& q, double& cost,std::vector<Eigen::Vector3d>& gradient);
};

#endif