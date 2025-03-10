#ifndef __POLYNOMINAL_TRAJ_H__
#define __POLYNOMINAL_TRAJ_H__

#include <vector>
#include <Eigen/Dense>

using std::vector;

class PolynomialTraj {
public:
    PolynomialTraj();
    ~PolynomialTraj();

    void init(vector<double> times);
    void reset(void);
    void addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t);
    Eigen::Vector3d evaluate_pos(double t);
    Eigen::Vector3d evaluate_vel(double t);
    Eigen::Vector3d evaluate_acc(double t);
    vector<Eigen::Vector3d> getTraj(void);
    double getJerk(void);

    static PolynomialTraj minSnapTraj(const Eigen::Vector3d &start_vel,const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_vel,const Eigen::Vector3d &end_acc,
                                    const Eigen::MatrixXd &pos,const Eigen::VectorXd &times);

    
private:
    vector<double> _times;
    vector<vector<double>> _coe_x_seg;
    vector<vector<double>> _coe_y_seg;
    vector<vector<double>> _coe_z_seg;

    vector<Eigen::Vector3d> _traj_3d;

    double _time_sum;
    int _num_seg;
};

#endif