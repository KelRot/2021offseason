// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  frc::CameraServer::GetInstance()->StartAutomaticCapture();

  prefs= frc::Preferences::GetInstance();
  enckP= prefs->GetDouble("enckP",0.0);
  enckI= prefs->GetDouble("enckI",0.0);
  enckD= prefs->GetDouble("enckD",0.0);

  gyrokP= prefs->GetDouble("gyrokP",0.0);
  gyrokI= prefs->GetDouble("gyrokI",0.0);
  gyrokD= prefs->GetDouble("gyrokD",0.0);

  pid.kP=enckP;
  pid.kI=enckI;
  pid.kD=enckD;

  gyropid.kP=gyrokP;
  gyropid.kI=gyrokI;
  gyropid.kD=gyrokD;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
   frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
   frc::SmartDashboard::PutNumber("Distance", enc.GetDistance());
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;
  enc.Reset();
  gyro.Reset();

}

void Robot::AutonomousPeriodic() {
  drive.ArcadeDrive(pid.computePID(enc.GetDistance(),360,50),-gyropid.computePID(gyro.GetAngle(),0,10));
}

void Robot::TeleopInit() {
  enc.Reset();
  gyro.Reset();
}

void Robot::TeleopPeriodic() {
  drive.CurvatureDrive(-js.GetRawAxis(1),js.GetRawAxis(4),js.GetRawButton(6));//butonlar ayarlanacak
  if(js.GetRawButton(5)){
    tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
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
