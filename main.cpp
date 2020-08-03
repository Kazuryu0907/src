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


Serial serial(USBTX, USBRX);

#define USE_ONLY_ODO

 double p = 0.001;
 double i = 0.001;
 double d = 0.001;
Timer TimerForQEI;

double L = 100;

double Wheelrad = 100;
#ifndef USE_ONLY_ODO
struct
{
  PinName IMUSDA = PB_11;
  PinName IMUSCL = PB_10;
} I2CPin;
#endif

PwmOut POA1 = PC_8; 
PwmOut POB1 = PC_9; 
PwmOut POA2 = PB_8; 
PwmOut POB2 = PB_9; 
PwmOut POA3 = PB_10; 
PwmOut POB3 = PB_11;

PwmOut OutPwms[6] = {POA1,POB1,POA2,POB2,POA3,POB3};


struct
{
  PinName APulse1 = PD_7;
  PinName BPulse1 = PD_6;
  PinName IndexPulse1 = NC;
  PinName APulse2 = PD_5;
  PinName BPulse2 = PD_4;
  PinName IndexPulse2 = NC;
  PinName APulse3 = PG_2;
  PinName BPulse3 = PG_3;
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



#ifndef USE_ONLY_ODO
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, 400000);
#endif
double dt = 0.01;
double a = 5;
double R = 30;

double Position[3] = {0,0,0};
double xr[3] = {0,0,0};

MPC mpc(dt,100,100);

double OmegaToRpm(double omega);
void solveN(double Nx,double Ny,double Nth,double theta,double L,double ret[3]);
double XtoN(double x);
double OtoN(double o);

MWodometry odos[3] = {
    MWodometry(encos[0],Odometrys.encoderPPR,Wheelrad),
    MWodometry(encos[1],Odometrys.encoderPPR,Wheelrad),
    MWodometry(encos[2],Odometrys.encoderPPR,Wheelrad)
  };
  int count_i = 0;
int main(){
    FOR(i,6)OutPwms[i].period_ms(1);
    double firstxr[16][3];
    FOR(i,16){
      firstxr[i][0] = a * cos(DeToRa(count_i*0.1));
      firstxr[i][1] = a * sin(DeToRa(count_i*0.1));
      firstxr[i][2] = 0;
      count_i++;
    }
    mpc.setxr(firstxr);
    #ifndef USE_ONLY_ODO
    IMU.setup();
    #endif
    double OutPwm[3];
    
    for(;;){
      #ifndef USE_ONLY_ODO
      IMU.update();
      Position[2] = IMU.getYaw();
      Position[2] = Position[2] * M_PI / 180;
      #endif
      
      #ifdef  USE_ONLY_ODO
      double allenco = 0;
      FOR(i,3){
        allenco += encos[i].getRPS()/30*M_PI;
        }
      allenco /= L;
      Position[2] = Position[2] + dt*allenco;
      #endif

      Position[0] = Position[0] + dt*M_PI/30*(odos[0].getDistance()*cos(Position[2]) + odos[1].getDistance()*cos(Position[2]+5/6*M_PI) + odos[2].getDistance()*cos(Position[2]+7/6*M_PI));
      Position[1] = Position[1] + dt*M_PI/30*(odos[0].getDistance()*sin(Position[2]) + odos[1].getDistance()*sin(Position[2]+5/6*M_PI) + odos[2].getDistance()*sin(Position[2]+7/6*M_PI));
      
      FOR(i,3)odos[i].setDistance(0);

      xr[0] = a * cos(DeToRa(count_i*0.1));
      xr[1] = a * sin(DeToRa(count_i*0.1));
      xr[2] = 0;

      mpc.updateAngle(Position[2]);
      mpc.solv(xr,Position);
      double *outu = mpc.getu();
      double Nrs[3];
      FOR(i,3)Nrs[i] = OmegaToRpm(outu[i]);
      mdd.update(Nrs);
      mdd.getRPMToPWM(OutPwm);
      FOR(i,3){
        OutPwms[i*2] = OutPwm[i] > 0 ? OutPwm[i] : 0;
        OutPwms[i*2+1] =  OutPwm[i] < 0 ? -OutPwm[i] : 0;
      }
      count_i++;
    }

    wait(dt);
}

double OmegaToRpm(double omega){
  return(30*omega/M_PI);
}