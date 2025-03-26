#include "polynomial_traj/polynomial_traj.h"

using std::vector;

void PolynomialTraj::init(vector<double> times)
{
    _times = times;
    _num_seg = times.size();
    _time_sum = 0.0;

    for(int i = 0;i < times.size();i++){
        _time_sum += _times[i];
    }
}

void PolynomialTraj::reset(void)
{
    _times.clear();
    _coe_x_seg.clear(),_coe_y_seg.clear(),_coe_z_seg.clear();
    _time_sum = 0.0;
    _num_seg = 0;
}

void PolynomialTraj::addSegment(vector<double> cx, vector<double> cy, vector<double> cz, double t)
{
    _times.push_back(t);
    _coe_x_seg.push_back(cx),_coe_y_seg.push_back(cy),_coe_z_seg.push_back(cz);
}

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::Vector3d &start_vel,const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &end_vel,const Eigen::Vector3d &end_acc,
                                    const Eigen::MatrixXd &pos,const Eigen::VectorXd &times)
{
    int seg_num = times.size();

    //order of polunomial
    int order = 6;
    // num of coeff:segnum * order * 3 (x y z)
    Eigen::MatrixXd poly_coeff(seg_num,3 * order);

    // fixed variables and free variables
    int num_f,num_p;
    // total derivatives constarint
    int num_d;

    const static auto getFactorial = [](int x){
        int fac = 1;
        for (int i = x;i > 0;i--){
            fac = fac * i;
        }

        return fac;
    };

    Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

    for(int k = 0;k < seg_num;k++){
        Dx(k * 6) = pos(k,0);
        Dx(k * 6 + 1) = pos(k+1,0);
        Dy(k * 6) = pos(k,1);
        Dy(k * 6 + 1) = pos(k+1,1);
        Dz(k * 6) = pos(k,2);
        Dz(k * 6 + 1) = pos(k+1,2);

        if(k == 0){
            Dx(k * 6 + 2) = start_vel(0);
            Dx(k * 6 + 4) = start_acc(0);

            Dy(k * 6 + 2) = start_vel(1);
            Dy(k * 6 + 4) = start_acc(1);
            
            Dz(k * 6 + 2) = start_vel(2);
            Dz(k * 6 + 4) = start_acc(2);
        } else if(k == seg_num - 1){
            Dx(k * 6 + 3) = end_vel(0);
            Dx(k * 6 + 5) = end_acc(0);

            Dy(k * 6 + 3) = end_vel(1);
            Dy(k * 6 + 5) = end_acc(1);
            
            Dz(k * 6 + 3) = end_vel(2);
            Dz(k * 6 + 5) = end_acc(2);
        }
    }

    Eigen::MatrixXd Ab;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for(int k = 0;k < seg_num;k++){
        Ab = Eigen::MatrixXd::Zero(6,6);
        for(int i = 0;i < 3;i++) {
            Ab(2 * i,i) = getFactorial(i);
            for(int j = i;j < 6;j++){
                Ab(2 * i + 1,j) = getFactorial(j) / getFactorial(j - i) * pow(times(k),j-i);
            }
            A.block(k * 6,k * 6,6,6) = Ab;
        }
    }

    Eigen::MatrixXd Ct,C;

    num_f = 2 * seg_num + 4;
    num_p = 2 * seg_num - 2;
    num_d = 6 * seg_num;
    
    Ct = Eigen::Matrix3d::Zero(num_d,num_f + num_p);

    // Dimensionality Reduction
    // Ct * [ d_f  = d
    //       d_p ]
    // d = [p0 p1 v0 v1 a1 a0 p1 p2 v1 v2 a1 a2 ...]
    // d_f = [p0 v0 a0 p1 p2 ··· pm vm am]
    // d_p = [v1 a1 v2 a2 v3 a3 ...] 
    // stack the start point
    Ct(0, 0) = 1; // p0 1 -> 1
    Ct(2, 1) = 1; // v0 3 -> 2
    Ct(4, 2) = 1; // a0 5 -> 3       
    Ct(1, 3) = 1;//  p1 2 -> 4
    Ct(3, 2 * seg_num + 4) = 1; // v1 4 ->  2 * s + 5 
    Ct(5, 2 * seg_num + 5) = 1; // a1 6 ->  2 * s + 6

    // stack the end point
    Ct(6 * (seg_num - 1) + 0,2 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 1,2 * seg_num + 1) = 1;
    Ct(6 * (seg_num - 1) + 2,4 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 3,2 * seg_num + 2) = 1;
    Ct(6 * (seg_num - 1) + 4,4 * seg_num + 1) = 1;
    Ct(6 * (seg_num - 1) + 5,2 * seg_num + 3) = 1;

    for(int j = 1;j < seg_num - 1;j++){
        Ct(6 * j + 0, 2 + j) = 1; 
        Ct(6 * j + 1, 2 + j + 1) = 1;
        Ct(6 * j + 2, 2 * seg_num + 4 + 2 * (j - 1)) = 1;
        Ct(6 * j + 3, 2 * seg_num + 4 + 2 * (j - 1) + 2) = 1;
        Ct(6 * j + 4, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
        Ct(6 * j + 5, 2 * seg_num + 4 + 2 * (j - 1) + 3) = 1;
    }

    C = Ct.transpose();
    
    // I don't know
    Eigen::VectorXd Dx_s(num_f + num_p),Dy_s(num_f + num_p),Dz_s(num_f + num_p);

    for(int k = 0;k <= seg_num;k++){
        if(k == 0){
            Dx_s(0) = pos(k,0);
            Dx_s(1) = start_vel(0);
            Dx_s(2) = start_acc(0);

            Dy_s(0) = pos(k,1);
            Dy_s(1) = start_vel(1);
            Dy_s(2) = start_acc(1);

            Dz_s(0) = pos(k,2);
            Dz_s(1) = start_vel(2);
            Dz_s(2) = start_acc(2);
        } else if(k == seg_num){
            Dx_s(2 * seg_num + 1) = pos(k,0);
            Dx_s(2 * seg_num + 2) = end_vel(0);
            Dx_s(2 * seg_num + 3) = end_acc(0);

            Dy_s(2 * seg_num + 1) = pos(k,1);
            Dy_s(2 * seg_num + 2) = end_vel(1);
            Dy_s(2 * seg_num + 3) = end_acc(1);

            Dz_s(2 * seg_num + 1) = pos(k,2);
            Dz_s(2 * seg_num + 2) = end_vel(2);
            Dz_s(2 * seg_num + 3) = end_acc(2);
        } else {
            Dx_s(2 + k) = pos(k,0);
            Dy_s(2 + k) = pos(k,1);
            Dz_s(2 + k) = pos(k,2);
        }
    }

    // minimum snap Matrix
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6,seg_num * 6);
    for(int k = 0;k < seg_num;k++){
        for(int i = 4;i < 6;i++){
            for(int j = 4;j < 6;j++){
                Q(k * 6 + i,k * 6 + j) = i * (i -1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (j - 3) /(i + j - 7) * std::pow(times(k),i + j - 7);
            }
        }
    }

    // R 
    Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;
    Eigen::VectorXd Dxf(2 * seg_num + 4),Dy(2 * seg_num + 4),Dz(2 * seg_num + 4);

    Eigen::MatrixXd Rff(2 * seg_num + 4,2 * seg_num + 4);
    Eigen::MatrixXd Rfp(2 * seg_num + 4,2 * seg_num - 2);
    Eigen::MatrixXd Rpf(2 * seg_num - 2,2 * seg_num + 4);
    Eigen::MatrixXd Rpp(2 * seg_num - 2,2 * seg_num - 2);

    Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
    Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
    Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
    Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

    // close form solution
    Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
    Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
    Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
    Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

    Dx_s.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
    Dy_s.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
    Dz_s.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

    Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);
    
    Px = (A.inverse() * Ct) * Dx_s;
    Py = (A.inverse() * Ct) * Dy_s;
    Pz = (A.inverse() * Ct) * Dz_s;

    for (int i = 0; i < seg_num; i++) {
        poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
    }

    PolynomialTraj poly_traj;

    for(int i = 0;i < poly_coeff.rows();++i){
        std::vector<double> cx(6),cy(6),cz(6);
        for (int j = 0; j < 6; ++j) {
            cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
        }
        reverse(cx.begin(), cx.end());
        reverse(cy.begin(), cy.end());
        reverse(cz.begin(), cz.end());

        double ts = times(i);
        poly_traj.addSegment(cx, cy, cz, ts);
    }

    return poly_traj;
}
