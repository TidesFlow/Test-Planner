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

