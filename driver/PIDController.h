
// 制御対象: controlled object.
// 制御量:   controlled variable.
// 目標値:   desired value.
// 操作量:   manipulated variable.

class PIDController
{

public:
  // enable all gain and manipulated variable.
  PIDController(double pGain, double iGain, double dGain)
  {
    this->pGain = pGain;
    this->iGain = iGain;
    this->dGain = dGain;
    this->enableI = true;
    this->enableD = true;
  }

  // enable only propotional gain and manipulated variable.
  PIDController(double pGain)
  {
    this->pGain = pGain;
    this->enableI = false;
    this->enableD = false;
  }

  void update(double target, double current);

  typedef enum gainType
  {
    proportional,
    integral,
    differential,
  } gainType;

  void modifyGain(gainType, double);
  void setOutputLimit(double);
  void setGeinLimit(gainType,double);
  double getTerm(){return(manipulatedVariable);}
  double getTerm(gainType);
  double getError(){return this->iGain;};
private:
  double pGain, iGain, dGain;
  bool enableI, enableD;

  // 毎回のターゲット値と現在値の差の蓄積(I制御で使用)
  double errorStack = 0;
  // 前回の偏差
  double prevError;
  // 容認する最大出力
  double maxValue;
  double manipulatedVariable;
  double proportionalManipulatedVariable;
  double integralManipulatedVariable;
  double differentialManipulatedVariable;
  double LimitErrorStack;
  double error;
  };
