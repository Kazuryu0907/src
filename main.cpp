#define _USE_MATH_DEFINES
#include "mbed.h"
#include "sensor/MPU9250.h"
#include "sensor/QEI.h"
//#include "driver/PIDController.h"
#include "driver/MWodometry.h"
#include "driver/WheelKinematics.h"
#include "driver/MDD.h"
#include "mpc_/MPC.h"

#include <math.h>

#define FOR(i,n) for(int (i) = 0;(i)<(n);(i)++)
#define DeToRa(i) (M_PI*(i)/0.00556)

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


QEI decoiQEI(NC,NC,NC,0,&TimerForQEI,QEI::X4_ENCODING);
QEI encos[4] = {encoder1,encoder2,encoder3,decoiQEI};
MDD mdd(encos,p,i,d,0.8);
MWodometry odos[3];
FOR(i,3)odos[i] = new MWodometry(&encos[i],Odometrys.encoderPPR,Wheelrad);

#ifndef USE_ONLY_ODO
MPU9250 IMU(I2CPin.IMUSDA, I2CPin.IMUSCL, 400000);
#endif
double dt = 0.01;
MPC mpc(dt,100,100);

double OmegaToRpm(double omega);
double Position[3] = {0,0,0};

int main(){
    FOR(i,6)OutPwms[i].period_ms(1);
    #ifndef USE_ONLY_ODO
    IMU.setup();
    #endif
    double xr[16][3];
    double a = 100;
    FOR(i,16){
      xr[i][0] = a*(DeToRa(i)-sin(DeToRa(i)));
      xr[i][1] = a*(1-cos(DeToRa(i)));
      xr[i][2] = 0;
    }
    mpc.setxr(xr);
    int i_count = 15+15;
    double OutPwm[4];
    for(;;){
      #ifndef USE_ONLY_ODO
      IMU.update();
      Position.[0] = IMU.getYaw();
      #endif
      #ifdef  USE_ONLY_ODO
      double allenco = 0;
      FOR(i,3){
        allenco += encos[i].getRPM()/30*M_PI;
        }
      allenco /= L;
      Position[0] = Position[0] + dt*allenco;
      #endif

      Position[1] = M_PI/30*(encos[0].getRPM()*cos(Position[0]) + encos[1].getRPM()*cos(Position[0]+5/6*M_PI) + encos[2].getRPM()*cos(Position[0]+7/6*M_PI));
      Position[2] = M_PI/30*(encos[0].getRPM()*sin(Position[0]) + encos[1].getRPM()*sin(Position[0]+5/6*M_PI) + encos[2].getRPM()*sin(Position[0]+7/6*M_PI));

      double oncexr[3];
      oncexr[0] = a*(DeToRa(i_count)-sin(DeToRa(i_count)));
      oncexr[1] = a*(1-cos(DeToRa(i_count)));
      oncexr[2] = 0;
      mpc.solv(oncexr,Position);
      double *solvedu;
      solvedu = mpc.getu();
      double TrRPM[4];
      FOR(i,3)TrRPM[i] = OmegaToRpm(solvedu[i]);
      mdd.update(TrRPM);
      mdd.getRPMToPWM(OutPwm);
      FOR(i,3){
        OutPwms[i*2] = Outpwm[i] > 0 ? Outpwm[i] : 0;
        OutPwms[i*2+1] =  Outpwm[i] < 0 ? -Outpwm[i] : 0;
      }
    }

    wait(dt);


    printf("\n");
    return(1);
}

double OmegaToRpm(double omega){
  return(30*omega/M_PI);
}