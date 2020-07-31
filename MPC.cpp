#include "MPC.h"
#include <iostream>
#define FOR(i,n) for(int (i) = 0;(i)<(n);(i)++)
#define DTR(n) ((n)*0.0174533)

MPC::MPC(double dt,double maxu,double L){
      _dt = dt;
      _MPCL = L;
      _maxu = maxu;
      set_defaults();
      setup_indexing();
      load_default_data();
      settings.verbose = 0;
}



void MPC::shiftArray(){
      FOR(i,15)FOR(k,3)params.xr[i][k] = params.xr[i+1][k];
}

void MPC::solv(double *xr,double *cx){
    FOR(i,3)params.xr[15][i] = xr[i];
    FOR(i,3)params.xzero[i] = cx[i];
    solve();
    FOR(i,16){
      FOR(k,3){
            //printf("%f:",vars.u[i][k]);
      }
      //printf("\n");
    }
    FOR(i,3)outu[i] = vars.u[0][i];
    double outx[3];
    FOR(i,3)outx[i] = vars.x[0][i];
    Eigen::Map<Eigen::Vector3d> uvec(&outu[0]);
    //std::cout << uvec << std::endl;
    x0 = x0 + _dt*B*uvec;
    FOR(i,3)params.xzero[i] = x0.data()[i];
    //std::cout << x0 << std::endl;
    printf("%f:%f:%f---------",params.xzero[0],params.xzero[1],params.xzero[2]);
    printf("%f:%f:%f\n",vars.u[0][0],vars.u[0][1],vars.u[0][2]);
    shiftArray();
}

void MPC::updateAngle(double theta){
  B << cos(theta),cos(theta+DTR(120)),cos(theta+DTR(240)),
       sin(theta),sin(theta+DTR(120)),sin(theta+DTR(240)),
       1/_MPCL,1/_MPCL,1/_MPCL;
}


void MPC::load_default_data(){
     /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  FOR(i,3)x0(i) = 0;

  params.a[0] = 1000;
  params.b[0] = 1000;
  params.dt[0] = _dt;
  params.maxthetaerror[0] = 1.0;

  MatrixXd A1(3,3);
  A1 << 1,0,0,
        0,1,0,
        0,0,0;
  params.A1 = A1.data();
  
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  MatrixXd C1(3,3);
  C1 << 0,0,0,
        0,0,0,
        0,0,1;

  //params.C1 = C1.data();
  //FOR(i,16)FOR(k,3)params.xr[i][k] = i*10+k;

  params.A[0] = 1;
  params.A[1] = 1;
  params.A[2] = 1;
  
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  B << 1,-0.5,-0.5,
       0,sqrt(3)/2,-sqrt(3)/2,
       1/_MPCL,1/_MPCL,1/_MPCL;

  params.B = B.data();

  Vector3d Vmaxu(_maxu,_maxu,_maxu);
  //printf("%f\n",Vmaxu(0));
  FOR(i,3)params.maxu[i] = Vmaxu.data()[i];
  Vector3d xzero(0,0,0);
 
  FOR(i,3)params.xzero[i] = xzero.data()[i];
  Vector3d Theta1(0,0,1);
  FOR(i,3)params.Theta1[i] = Theta1.data()[i]; 
  }

void MPC::setxr(double xr[16][3]){
      FOR(i,16)FOR(k,3)params.xr[i][k] = xr[i][k];
}




void MPC::print(){
      printf("%f:%f:%f------",vars.x[0][0],vars.x[0][1],vars.x[0][2]);
      printf("%f:%f:%f------",vars.u[0][0],vars.u[0][1],vars.u[0][2]);
      printf("%f:%f:%f\n",params.xr[0][0],params.xr[0][1],params.xr[0][2]);
}
