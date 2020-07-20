#include "MDD.h"

/*
void MDD::setPWMLimit(double l)
{
    LimitPWM = l;
}
*/
void MDD::update(double *ReceiveRPM){
  for(int i = 0;i<4;i++){
      RPMs[i] = abs(QEIs[i]->getRPM());
      PIDs[i]->update(*(ReceiveRPM+i),RPMs[i]);
  }
}

void MDD::getRPMToPWM(double *array)
{
    for(int i = 0;i<4;i++)array[i] = PIDs[i]->getTerm();
}

void MDD::getCurrentRPM(double *array)
{
    for(int i = 0;i<4;i++)array[i] = RPMs[i];
}

void MDD::getPIDParams(int i,double *array)
{
    array[0] = PIDs[i]->getTerm(PIDController::proportional);
    array[1] = PIDs[i]->getTerm(PIDController::integral);
    array[2] = PIDs[i]->getTerm(PIDController::differential);
}
/*
void MDD::sendPwmAverage(UnitProtocol *u)
{
    double ave = 0;
    for(int i = 0;i<4;i++)ave += RPMs[i];
    ave *= 0.25;
    uint8_t sendpacket[2];
    uint16_t temp = (uint16_t)ave*10;
    sendpacket[0] = temp >> 8;
    sendpacket[1] = temp & 0b11111111;
    u->transmit(2,sendpacket);
}
*/