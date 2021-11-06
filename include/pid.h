#include "main.h"

class pidController{
  public:
    //Constructors
    pidController(){}
    pidController(double targetValue):
     tVal(targetValue)
    {}
    pidController(double targetValue, double pGain, double iGain, double dGain):
     tVal(targetValue), kP(pGain), kI(iGain), kD(dGain)
    {
      iLim = maxLim; ///2;
    }
    pidController(double targetValue, double pGain, double iGain, double dGain, double integralLimit, double maxLimit, double minLimit):
     tVal(targetValue), kP(pGain), kI(iGain), kD(dGain), iLim(integralLimit), maxLim(maxLimit), minLim(minLimit)
    {}
    //Pid calculations
    void update(double inVal){
      error = tVal - inVal;
      integral += error;
      derivative = error - lasterror;
      lasterror = error;
    }
    //Check if values are out of range;
    void limitCases(){
      if(fabs(out) > maxLim) out = getSign(out) * maxLim;
      if(fabs(out) < minLim) out = getSign(out) * minLim;
      if(fabs(integral) > iLim) integral = getSign(integral) * iLim;
    }
    //Calculate output, stop if near target
    double calculateOut(){
      out = kP * error + kI * integral + kD * derivative;
      limitCases();
      if(withinTarget()){
        out = 0;
      }
      return out;
    }
    void updateTarget(double newTarget){
      tVal = newTarget;
    }
    void resetID(){
      integral = 0;
      derivative = 0;
      lasterror = 0;
    }
    bool withinTarget(){
      if(abs(error) > tolerance){
        integral = 0;
        return false;
      }
      return true;
    }
    double abs(double input){
      if(input > 0) return input;
      return -input;
    }
    double getSign(double input){
      //if(input == 0) return 0;
      return abs(input)/input;
    }
    //Variables
    const double kP = 0;
    const double kI = 0;
    const double kD = 0;

    double tVal = 0;

    double maxLim = 200;
    double minLim = 20;
    double iLim;

    double error = 0;
    double integral = 0;
    double derivative = 0;
    double lasterror = error;
    double out = 0;

    double tolerance = 1;
};
