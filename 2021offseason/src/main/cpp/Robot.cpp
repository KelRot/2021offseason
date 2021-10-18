
#include "Robot.h"

frc::Preferences *prefs;

double enckP;
double enckI;
double enckD;

double gyrokP;
double gyrokI;
double gyrokD;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  enc.SetDistancePerPulse((15*3.1415)/1024);
  gyro.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kX);

  frc::CameraServer::GetInstance()->StartAutomaticCapture();

  frc::SmartDashboard::PutBoolean("Drive Reversed",false);

}


void Robot::RobotPeriodic() {
   frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
   frc::SmartDashboard::PutNumber("Distance", enc.GetDistance());
   frc::SmartDashboard::PutNumber("enc error", encpid.piderror);
   std::cout<<gyro.GetAngle()<<std::endl;

  

  prefs= frc::Preferences::GetInstance();
  enckP= prefs->GetDouble("enckP",0.0);
  enckI= prefs->GetDouble("enckI",0.0);
  enckD= prefs->GetDouble("enckD",0.0);

  gyrokP= prefs->GetDouble("gyrokP",0.0);
  gyrokI= prefs->GetDouble("gyrokI",0.0);
  gyrokD= prefs->GetDouble("gyrokD",0.0);

  encpid.kP=enckP;
  encpid.kI=enckI;
  encpid.kD=enckD;

  gyropid.kP=gyrokP;
  gyropid.kI=gyrokI;
  gyropid.kD=gyrokD;

}


void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
 
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  enc.Reset();
  gyro.Reset();
  driveReversed=false;

    phase = 1;
    t.Start();
    startPoint = 0; //shuffleboarddan alinacak
    ang = atan((305.0 - c) / startPoint);
    d = sqrt((305.0 - c) * (305.0 - c) + startPoint * startPoint); 

}

void Robot::AutonomousPeriodic() {
  drive.ArcadeDrive(0,-gyropid.computePID(gyro.GetAngle(),90,5)); //tuning

   /* dist = angle = 0;
    if(phase == 1){
        dist = 0;
        angle = 90.0 - ang;
        if(t.Get() > donusSuresi || abs(90 - gyro.GetAngle()) < 2){
            t.Reset();
            ++phase;
        }
    }else if(phase == 2){
        dist = d;
        angle = 0;
        if(t.Get() > gidisSuresi || abs(dist - enc.GetDistance()) < 2){
            t.Reset();
            ++phase;
        }
    }else if(phase == 3){
        dist = 0;
        angle = -(90.0 - ang);
        if(t.Get() > donusSuresi || abs(- gyro.GetAngle()) < 2){
            t.Reset();
            ++phase;
        }
    }else if(phase == 4){   
        dist = c;
        angle = 0;
        if(t.Get() > gidisSuresi || abs(dist - enc.GetDistance()) < 2){
            t.Reset();
            ++phase;
        }
    }
    drive.ArcadeDrive(encpid.computePID(enc.GetDistance(), dist, 50), -gyropid.computePID(gyro.GetAngle(), angle, 10));*/
}

void Robot::TeleopInit() {
  enc.Reset();
  gyro.Reset();
  driveReversed=false;//Default intake ön
}

void Robot::TeleopPeriodic() {
  if(js.GetRawButtonPressed(4)){
    driveReversed= !driveReversed; //Drive mode switched
  }
  if(driveReversed){
    drive.CurvatureDrive(js.GetRawAxis(1),js.GetRawAxis(4),js.GetRawButton(5));
    frc::SmartDashboard::PutBoolean("Drive Reversed",driveReversed);
  }
  else{
    drive.CurvatureDrive(-js.GetRawAxis(1),js.GetRawAxis(4),js.GetRawButton(5));
    frc::SmartDashboard::PutBoolean("Drive Reversed",driveReversed);
  }

  if(js.GetRawButton(6)){
    tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  }
  if(js.GetRawAxis(3)==1){
    tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
