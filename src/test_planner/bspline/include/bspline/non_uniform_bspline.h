#ifndef __NON_UNIFORM_BSPLINE
#define __NON_UNIFORM_BSPLINE

#include <Eigen/Dense>

#include <vector>
#include <iostream>

class NonUniformBSpline {
public:
    NonUniformBSpline(){}
    NonUniformBSpline(const Eigen::MatrixXd &control_points,const int &order,const double &interval);

    ~NonUniformBSpline();

    /* --------------- parameterize function ------------- */
    void parameterizeToBspline(std::vector<Eigen::Vector3d> points_set,
                                std::vector<Eigen::Vector3d> start_end_derivatives,
                                double ts,
                                Eigen::MatrixXd &control_points);
    /* --------------- some physical function ------------ */
    bool checkFeasilbility(bool show);
    double checkRatio(void);
    bool reallocateTime(bool show);
    /* --------------- some calculate ----------- */
    Eigen::VectorXd evaluateDedoor(const double &u);
    Eigen::MatrixXd getDerivativePoints(void);
    NonUniformBSpline getDerivative(void);
    /* --------------- some property ------------ */
    void setNonUniformBSpline(const Eigen::MatrixXd &control_points,const int &order,const double &interval);
    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot(void);
    void getTimeSpan(double &up,double &um_p);
    Eigen::VectorXd getControlPoints(void);
    double getInterval(void);
    void setPhysicalLimits(const double& vel,const double& acc);
private:
    Eigen::MatrixXd _control_points;
    Eigen::MatrixXd _u; // knot vector

    double _interval;
    int _p,_n,_m; //p degeree n control points num m kont vector size

    double _limit_ratio_;
    double _limit_vel,_limit_acc;

};

#endif