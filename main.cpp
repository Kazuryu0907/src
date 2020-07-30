#define _USE_MATH_DEFINES
#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
//#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"
#include "driver/MDD.h"

#include "MPC.h"

#include "Eigen/Core"
#include "Eigen/LU"

#include <math.h>

using namespace Eigen;

#define FOR(i,n) for(int (i) = 0;(i)<(n);(i)++)
#define DeToRa(i) (M_PI*(i)/0.00556)

Params params;
Settings settings;
Vars vars;
Workspace work;

MPC mpc(0.01,100,100);
Serial serial(USBTX, USBRX);

#define USE_ONLY_ODO
/*
PIDController pidObX(0.1, 0.05, 0);
PIDController pidObY(0.1, 0.05, 0);
PIDController pidObYaw(0.01, 0.005, 0);
*/
 double p = 0.001;
 double i = 0.001;
 double d = 0.001;
Timer TimerForQEI;

double L = 100;

double Wheelrad = 100;
struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;

PwmOut POA1 = NC; 
PwmOut POB1 = NC; 
PwmOut POA2 = NC; 
PwmOut POB2 = NC; 
PwmOut POA3 = NC; 
PwmOut POB3 = NC;

PwmOut OutPwms[6] = {POA1,POB1,POA2,POB2,POA3,POB3};


struct
{
  PinName APulse1 = PF_9;
  PinName BPulse1 = PF_8;
  PinName IndexPulse1 = NC;
  PinName APulse2 = PA_4_ALT0;
  PinName BPulse2 = PB_0_ALT0;
  PinName IndexPulse2 = NC;
  PinName APulse3 = NC;
  PinName BPulse3 = NC;
  PinName IndexPulse3 = NC;
  const int encoderPPR = 1024;
} Odometrys;

QEI encoder1(Odometrys.APulse1,
                 Odometrys.BPulse1,
                 Odometrys.IndexPulse1,
                 Odometrys.encoderPPR,
                 &TimerForQEI,
                 QEI::X4_ENCODING);

QEI encoder2(Odometrys.APulse2,
                 Odometrys.BPulse2,
                 Odometrys.IndexPulse2,
                 Odometrys.encoderPPR,
                 &TimerForQEI,
                 QEI::X4_ENCODING);

QEI encoder3(Odometrys.APulse3,
                 Odometrys.BPulse3,
                 Odometrys.IndexPulse3,
                 Odometrys.encoderPPR,
                 &TimerForQEI,
                 QEI::X4_ENCODING);


QEI encos[3] = {encoder1,encoder2,encoder3};
MDD mdd(encos,p,i,d,0.8);

//FOR(i,3)odos[i] = new MWodometry(&encos[i],Odometrys.encoderPPR,Wheelrad);

#ifndef USE_ONLY_ODO
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, 400000);
#endif
double dt = 0.01;
double a = 5;
double R = 30;

double Position[3] = {0,0,0};
double xr[3] = {0,0,0};


double OmegaToRpm(double omega);
void solveN(double Nx,double Ny,double Nth,double theta,double L,double ret[3]);
double XtoN(double x);
double OtoN(double o);

MWodometry odos[3] = {
    MWodometry(encos[0],Odometrys.encoderPPR,Wheelrad),
    MWodometry(encos[1],Odometrys.encoderPPR,Wheelrad),
    MWodometry(encos[2],Odometrys.encoderPPR,Wheelrad)
  };
  int count_i = 16;
int main(){
    FOR(i,6)OutPwms[i].period_ms(1);
    double firstxr[16][3];
    FOR(i,16){
      firstxr[i][0] = a * cos(DeToRa(i*0.1));
      firstxr[i][1] = a * sin(DeToRa(i*0.1));
      firstxr[i][2] = 0;
    }
    mpc.setxr(firstxr);
    #ifndef USE_ONLY_ODO
    IMU.setup();
    #endif
    double OutPwm[3];
    for(;;){
      #ifndef USE_ONLY_ODO
      IMU.update();
      Position[0] = IMU.getYaw();
      Position[0] = Position[0] * M_PI / 180;
      #endif
      #ifdef  USE_ONLY_ODO
      double allenco = 0;
      FOR(i,3){
        allenco += encos[i].getRPM()/30*M_PI;
        }
      allenco /= L;
      Position[0] = Position[0] + dt*allenco;
      #endif

      Position[1] = Position[1] + dt*M_PI/30*(encos[0].getRPM()*cos(Position[0]) + encos[1].getRPM()*cos(Position[0]+5/6*M_PI) + encos[2].getRPM()*cos(Position[0]+7/6*M_PI));
      Position[2] = Position[2] + dt*M_PI/30*(encos[0].getRPM()*sin(Position[0]) + encos[1].getRPM()*sin(Position[0]+5/6*M_PI) + encos[2].getRPM()*sin(Position[0]+7/6*M_PI));
      

      xr[0] = a * cos(DeToRa(count_i*0.1));
      xr[1] = a * sin(DeToRa(count_i*0.1));
      xr[2] = 0;
      //FOR(i,3)TrRPM[i] = OmegaToRpm(solvedu[i]);
      double Nrs[3];
      //FOR(i,2)Nrs[i] = XtoN(xr[i]-Position[i]);
      mpc.solv(xr,Position);
      Nrs[2] = OtoN(xr[2]-Position[2]);
      double outNs[3];
      solveN(Nrs[0],Nrs[1],Nrs[2],Position[2],L,outNs);

      mdd.update(Nrs);
      mdd.getRPMToPWM(OutPwm);
      FOR(i,3){
        OutPwms[i*2] = OutPwm[i] > 0 ? OutPwm[i] : 0;
        OutPwms[i*2+1] =  OutPwm[i] < 0 ? -OutPwm[i] : 0;
      }
      count_i++;
    }

    //wait(dt);


    printf("\n");
}

double OmegaToRpm(double omega){
  return(30*omega/M_PI);
}

double XtoN(double x){
  double N = 30/M_PI/R/dt*x;
  return(N);
}

double OtoN(double o){
  return(30*o/M_PI/dt);
}
void solveN(double Nx,double Ny,double Nth,double theta,double L,double ret[3]){
    Matrix3d m;
    m << cos(DeToRa(theta)),cos(DeToRa(120+theta)),cos(DeToRa(240+theta)),
         sin(DeToRa(theta)),sin(DeToRa(120+theta)),sin(DeToRa(240+theta)),
         1/L,1/L,1/L;
    //std::cout << m << std::endl;
    double detA = m.determinant();
    Vector3d v(Nx,Ny,Nth);
    Matrix3d backm;
    backm = m;
    FOR(i,3){
        FOR(k,3){
            m(k,i) = v(k);
        }
        //std::cout << m << std::endl;
        //printf("----------------\n");
        ret[i] = m.determinant()/detA;
        m = backm;
    }
}