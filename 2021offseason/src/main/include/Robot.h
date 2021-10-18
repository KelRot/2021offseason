// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "PIDController.h"
#include "Include.h"
#include <ctre/Phoenix.h>
class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

  frc::Joystick js{0};

  frc::ADIS16470_IMU gyro{};

  frc::Talon solOn{0};
  frc::Talon solArka{1};
  frc::Talon sagOn{2};
  frc::Talon sagArka{3};
  frc::Encoder enc{0,1,true};

  frc::SpeedControllerGroup sol{solOn,solArka};
  frc::SpeedControllerGroup sag{sagOn,sagArka};

  frc::DifferentialDrive drive{sol,sag};

  PIDController encpid{0.047,0.0,0.0};
  PIDController gyropid{0.027,0.0,0.24};

  VictorSPX tirmanma{2};
  
  bool driveReversed=false;

  double startPoint, ang, d, c; //baslangic noktasi, aci, mesafe, son mesafe

  const double donusSuresi = 1.0, gidisSuresi = 4.0;
  //gidisK olarak degistirilecek
  int phase = 0;
  frc::Timer t;

  double dist, angle;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
