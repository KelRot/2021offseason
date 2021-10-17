#include <iostream>
#include <math.h>

class PIDController {
    private:
    
    public:
        double kP;
        double kI;
        double kD;
        PIDController(double kp,double ki,double kd){
            kP=kp;
            kI=ki;
            kD=kd;
        }
        double piderror=0;
        double preverror=0;
        double Ierror=0;
        double computePID(double current, double setpoint, double minerror){
            piderror= setpoint-current;
            double errord=piderror-preverror;
            if(abs(piderror)>minerror){
                Ierror=Ierror + piderror;
            }
            else{
                Ierror=0;
            }
            double comp= kP*piderror + kI*Ierror + kD*errord;

        return comp;
        }

};