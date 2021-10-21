#include <bits/stdc++.h>

class PIDController
{
    private:
    
    public:
        double kP, kI, kD;

        PIDController(double kp, double ki, double kd)
        {
            kP = kp, kI = ki, kD = kd;
        }

        double piderror = 0, preverror = 0, errord = 0, Ierror = 0;
       

        double computePID(double current, double setpoint, double minerror)
        {
            
            piderror = setpoint - current;
            errord = piderror - preverror;
            
            if(abs(piderror) < minerror)
                Ierror = Ierror + piderror;
            else
                Ierror = 0;

            if(abs(piderror) < 1)
                Ierror = 0;

            double comp = kP * piderror + kI * Ierror + kD * errord;
            preverror = piderror;
            return comp;
        }
};
