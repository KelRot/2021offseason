// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <Include.h>

class Robot : public frc::TimedRobot
{
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
        frc::Talon sagOn{3};
        frc::Talon sagArka{2};
        frc::Encoder enc{0, 1, 1};

        frc::SpeedControllerGroup sol{solOn, solArka};
        frc::SpeedControllerGroup sag{sagOn, sagArka};

        frc::DifferentialDrive drive{sol, sag};

        PIDController encpid{0.047, 0.0, 0.0};
        PIDController gyropid{0.027, 0.0, 0.24};

        VictorSPX tirmanma{0};
        VictorSPX tirmanma2{3};
        VictorSPX kapak{2};
        VictorSPX intex{1};

        frc::DigitalInput limitSwitchmin{4};

        //frc::DigitalInput limitSwitchmax{3};
    
        //frc::DigitalInput irsensor{2};

        bool driveReversed = 0;

        double startPoint, ang, d, c; //baslangic noktasi, aci, mesafe, son mesafe

        const double donusSuresi = 2.0, gidisSuresi = 4.0;
        int phase = 0;
        frc::Timer t;

        double dist, angle;

    private:
        frc::SendableChooser<string> m_chooser;
        const string kAutoNameDefault = "Default";
        const string kAutoNameCustom = "My Auto";
        string m_autoSelected;
};
