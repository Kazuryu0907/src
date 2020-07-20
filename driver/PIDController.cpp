#include "PIDController.h"

// 制御対象: controlled object.
// 制御量:   controlled variable.
// 目標値:   desired value.
// 操作量:   manipulated variable.

void PIDController::update(double target, double current)
{

  // 偏差
  error = target - current;
  errorStack += error;

  errorStack = errorStack > LimitErrorStack ? LimitErrorStack : errorStack < -LimitErrorStack ? -LimitErrorStack : errorStack;  

  // P制御の操作量
  proportionalManipulatedVariable = error * pGain;

  // I制御の操作量
  integralManipulatedVariable = errorStack * iGain;
  
  // D制御の操作量
  differentialManipulatedVariable = (error - prevError) * dGain;

  // 合計操作量
  manipulatedVariable = proportionalManipulatedVariable;
  if (enableI)
  {
    manipulatedVariable += integralManipulatedVariable;
  }
  if (enableD)
  {
    manipulatedVariable += differentialManipulatedVariable;
  }

  //constrain
  manipulatedVariable > maxValue ? manipulatedVariable = maxValue : manipulatedVariable < -maxValue ? manipulatedVariable = -maxValue : manipulatedVariable;

  //次回計算用に今回の偏差値を格納
  prevError = error;

}

double PIDController::getTerm(PIDController::gainType type)
{
  switch(type)
  {
    case proportional:
      return proportionalManipulatedVariable;
    break;
    case integral:
      return integralManipulatedVariable;
    break;
    case differential:
      return differentialManipulatedVariable;
    break;
  }
}

void PIDController::modifyGain(gainType parameter, double value)
{
  switch (parameter)
  {
  case proportional:
    pGain = value;
    break;
  case integral:
    iGain = value;
    break;
  case differential:
    dGain = value;
    break;
  }
}

void PIDController::setOutputLimit(double estimateValue)
{
  this->LimitErrorStack = estimateValue/this->iGain;
  maxValue = estimateValue;
}