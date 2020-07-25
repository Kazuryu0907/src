#include "C:\eigen\Eigen/Core"
using namespace Eigen;
#include "solver.h"

class MPC
{

public:
    MPC(double dt,double maxu,double L);
    void solv(double *xr,double *cx);
    void setxr(double xr[16][3]);
    double* getu(){return(outu);};
    void print();
    Vector3d x0;
    Matrix3d B;
    
private:
    Params params;
    Settings settings;
    Vars vars;
    double _dt;
    double _MPCL;
    double _maxu;
    double outu[3];
    void load_default_data(double maxu);
    void shiftArray();
};