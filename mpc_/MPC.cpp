#include "MPC.h"

#define FOR(i,n) for(int (i) = 0;(i)<(n);(i)++)


void shiftArray(){
      FOR(i,15)FOR(k,3)params.xr[i][k] = params.xr[i+1][k];
      /*
      double *temp[16];
      std::memmove(temp,&params.xr[1],60);
      std::memmove(params.xr,temp,60);
      double tempxr[3] = {0,0,0};
      params.xr[15] = tempxr;
      */
      //FOR(i,16)printf("%f:%f:%f\n",params.xr[i][0],params.xr[i][1],params.xr[i][2]);
}

void MPC::solv(double *xr,double *cx){
    params.xr[15] = xr;
    FOR(i,3)params.xzero[i] = cx[i];
    //print();
    //printf("%f:%f:%f\n",params.xr[15][0],params.xr[15][1],params.xr[15][2]);
    solve();
    FOR(i,3)outu[i] = vars.u[0][i];
    Eigen::Map<Eigen::Vector3d> uvec(&outu[0]);
    x0 = x0 + dt*B*uvec;
    FOR(i,3)params.xzero[i] = x0.data()[i];
    shiftArray();
}


void MPC::load_default_data(double maxu){
     /* Make this a diagonal PSD matrix, even though it's not diagonal. */
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

  params.C1 = C1.data();
  FOR(i,16)FOR(k,3)params.xr[i][k] = i*10+k;

  params.A[0] = 1;
  params.A[1] = 1;
  params.A[2] = 1;
  
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  B << 1,-0.5,-0.5,
       0,sqrt(3)/2,-sqrt(3)/2,
       1/L,1/L,1/L;

  params.B = B.data();

  Vector3d Vmaxu(maxu,maxu,maxu);
  printf("%f\n",Vmaxu(0));
  FOR(i,3)params.maxu[i] = Vmaxu.data()[i];
  Vector3d xzero(0,0,0);
 
  FOR(i,3)params.xzero[i] = xzero.data()[i];
}

void MPC::setxr(double xr[16][3]){
      FOR(i,16)FOR(k,3)params.xr[i][k] = xr[i][k];
}




void MPC::print(){
      //FOR(i,16)printf("%f:%f:%f\n",vars.x[i][0],vars.x[i][1],vars.x[i][2]);
      //printf("%f:%f:%f\n",vars.u[0][0],vars.u[0][1],vars.u[0][2]);
      printf("%f:%f:%f------",vars.x[0][0],vars.x[0][1],vars.x[0][2]);
      printf("%f:%f:%f------",vars.u[0][0],vars.u[0][1],vars.u[0][2]);
      printf("%f:%f:%f\n",params.xr[0][0],params.xr[0][1],params.xr[0][2]);
}