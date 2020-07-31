#include "C:\eigen\Eigen/Core"
using namespace Eigen;
extern "C" {
#include "solver.h"
}

class MPC
{

public:
    MPC(double dt,double maxu,double L);
    void solv(double *xr,double *cx);
    void setxr(double xr[16][3]);
    void updateAngle(double theta);
    double* getu(){return(vars.u[0]);};
    void print();
    
    Matrix3d B;
    double _dt;
    double _MPCL;
    double _maxu;
private:
    Vector3d x0;
    double outu[3];
    void load_default_data();
    void shiftArray();
};