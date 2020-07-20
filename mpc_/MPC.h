#include "solver.h"
#include "C:\eigen\Eigen/Core"
using namespace Eigen;

class MPC
{

public:
    MPC(double dt,double maxu,double L):dt(dt),L(L),maxu(maxu){
        set_defaults();
        setup_indexing();
        load_default_data(maxu);
        settings.verbose = 0;
    };

    void solv(double *xr,double *cx);
    void setxr(double xr[16][3]);
    double* getu(){return(outu);};
    void print();
    Vector3d x0;
    Matrix3d B;
private:
    double dt;
    double L;
    double maxu;
    double outu[3];
    void load_default_data(double maxu);
};