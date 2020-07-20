#include "mbed.h"
#include "PIDController.h"
#include "../sensor/QEI.h"
//#include "../CommunicationSource/UnitProtocol.hpp"

class MDD
{
    public:
    /*
    *@param Pin*8,pulsesPerRev,pGein,iGein,dGein,MaxPwm
    */
        MDD(PinName A1,PinName B1,PinName A2,PinName B2,PinName A3,PinName B3,PinName A4,PinName B4,int pls,double p,double i,double d,double max)
        {
            this->p = p;
            this->i = i;
            this->d = d;
            QEIs[0] = new QEI(A1,B1,NC,pls,&TimerForEnc);
            QEIs[1] = new QEI(A2,B2,NC,pls,&TimerForEnc);
            QEIs[2] = new QEI(A3,B3,NC,pls,&TimerForEnc);
            QEIs[3] = new QEI(A4,B4,NC,pls,&TimerForEnc);
            for(int i = 0;i<4;i++)PIDs[i] = new PIDController(this->p,this->i,this->d);
            for(int i = 0;i<4;i++)PIDs[i]->setOutputLimit(max);
        };
        MDD(QEI *qeis,double p,double i,double d,double max)
        {
            this->p = p;
            this->i = i;
            this->d = d;
            for(int i = 0;i<4;i++)QEIs[i] = &qeis[i];
            for(int i = 0;i<4;i++)PIDs[i] = new PIDController(this->p,this->i,this->d);
            for(int i = 0;i<4;i++)PIDs[i]->setOutputLimit(max);
        };
        void update(double *ReceiveRPM);
        void getRPMToPWM(double *array);
        void getCurrentRPM(double *array);
        void getPIDParams(int,double *array);
        //void sendPwmAverage(UnitProtocol*);
        double getError(){return PIDs[0]->getError();};
        //void setPWMLimit(double);
    private:
        Timer TimerForEnc;
        double RPMs[4];
        double PWMs[4];
        double p;
        double i;
        double d;
        //double LimitPWM;
        QEI *QEIs[4];
        PIDController *PIDs[4];
};

