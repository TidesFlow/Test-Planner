#ifndef __EDT_ENVIRONMENT_H__
#define __EDT_ENVIRONMENT_H__

class EDTEnvironment {
public:
    typedef EDTEnvironment* Ptr;

    void evaluateEDTWithGrad(const Eigen::Vector3d& pos, double time,double &dist, Eigen::Vector3d &grad);
};

#endif