
#include <Robot.h>

frc::Preferences *prefs;
double enckP, enckI, enckD;

double gyrokP, gyrokI, gyrokD;

void Robot::RobotInit()
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    enc.SetDistancePerPulse((15 * 3.1415) / 1024);
    gyro.SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kX);

    frc::CameraServer::GetInstance() -> StartAutomaticCapture();
    frc::CameraServer::GetInstance() -> StartAutomaticCapture();

    frc::SmartDashboard::PutBoolean("Drive Reversed", 0);

    frc::SmartDashboard::PutNumber("Starting Point", 0.0);
    startPoint = frc::SmartDashboard::GetNumber("Starting Point", 0.0);
}

void Robot::RobotPeriodic() 
{
    frc::SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());
    frc::SmartDashboard::PutNumber("Distance", enc.GetDistance());
    frc::SmartDashboard::PutNumber("enc error", encpid.piderror);
    

    /*prefs = frc::Preferences::GetInstance();
    enckP = prefs -> GetDouble("enckP", 0.0);
    enckI = prefs -> GetDouble("enckI", 0.0);
    enckD = prefs -> GetDouble("enckD", 0.0);

    gyrokP = prefs -> GetDouble("gyrokP", 0.0);
    gyrokI = prefs -> GetDouble("gyrokI", 0.0);
    gyrokD = prefs -> GetDouble("gyrokD", 0.0);

    encpid.kP = enckP;
    encpid.kI = enckI;
    encpid.kD = enckD;

    gyropid.kP = gyrokP;
    gyropid.kI = gyrokI;
    gyropid.kD = gyrokD; */
}

void Robot::AutonomousInit() 
{
    enc.Reset();
    gyro.Reset();
    driveReversed = 0;
    m_autoSelected = m_chooser.GetSelected();
    m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    cout << "Auto selected: " << m_autoSelected << endl;

    if (m_autoSelected == kAutoNameCustom) 
    {
        phase = 1;
        t.Start();
        startPoint = 0; //shuffleboarddan alinacak
        ang = atan((305.0 - c) / startPoint);
        d = sqrt((305.0 - c) * (305.0 - c) + startPoint * startPoint); 
    }
    else 
    {
        // Default Auto goes here
    } 
}

void Robot::AutonomousPeriodic() 
{
    if (m_autoSelected == kAutoNameCustom) 
    {
        dist = angle = 0;
        if(phase == 1)
        {
            dist = 0;
            angle = 90.0 - ang;
            if(t.Get() > donusSuresi || abs(angle - gyro.GetAngle()) < 2 || abs(gyropid.errord) < 0.5)
            {
                t.Reset();
                ++phase;
            }
        }else if(phase == 2)
        {
            dist = d;
            angle = 0;
            if(t.Get() > gidisSuresi || abs(dist - enc.GetDistance()) < 2 || encpid.errord==0)
            {
                t.Reset();
                ++phase;
            }
        }else if(phase == 3)
        {
            dist = 0;
            angle = -(90.0 - ang);
            if(t.Get() > donusSuresi || abs(angle - gyro.GetAngle()) < 2 || abs(gyropid.errord) < 0.5)
            {
                t.Reset();
                ++phase;
            }
        }else if(phase == 4)
        {   
            dist = c;
            angle = 0;
            if(t.Get() > gidisSuresi || abs(dist - enc.GetDistance()) < 2 || encpid.errord==0)
            {
                t.Reset();
                ++phase;
            }
        }else if(phase == 5)
        {
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
            if(t.Get() > 3)
                ++phase;
        }
        drive.ArcadeDrive(encpid.computePID(enc.GetDistance(), dist, 50), -gyropid.computePID(gyro.GetAngle(), angle, 10));
    }
    else
    {
        /*if(phase == 1)
        {
            drive.ArcadeDrive(encpid.computePID(enc.GetDistance(), 305, 50), gyropid.computePID(gyro.GetAngle(), 0, 10));
            if(t.Get() > 5 || abs(305 - enc.GetDistance()) < 2)
            {
                ++phase;
            }
        }
        else if(phase == 2)
        {
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
            if(t.Get() < 1)
            {
                kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
            }
        }
        
    */
    drive.ArcadeDrive( -encpid.computePID(enc.GetDistance(),265,50) , -gyropid.computePID(gyro.GetAngle(), -2 ,10) );
    }
}

void Robot::TeleopInit() 
{
    enc.Reset();
    gyro.Reset();
    driveReversed = 0;//Default intake ön
}

void Robot::TeleopPeriodic()
{
    //-------------------------------------------DRIVE-------------------------------------------

    if(js.GetRawButtonPressed(8))
        driveReversed ^= 1; //Drive mode switched
    
    if(driveReversed)
    {
        drive.CurvatureDrive(js.GetRawAxis(1), js.GetRawAxis(4), js.GetRawButton(5)); //Hazne ön //DIKKAT ET JOYSTICK DEGISTI
        frc::SmartDashboard::PutBoolean("Drive Reversed", driveReversed);            
    }
    else
    {
        drive.CurvatureDrive(-js.GetRawAxis(1), js.GetRawAxis(4), js.GetRawButton(5)); //Intex ön
        frc::SmartDashboard::PutBoolean("Drive Reversed", driveReversed);
    }

    //-------------------------------------------TIRMANMA-------------------------------------------
    if(js.GetRawButton(2))
    {
        tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.75); //tırmanma yukarı
        tirmanma2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.75);
    }
    else if(js.GetRawButton(6))
    {
        tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.75); //tırmanma aşağı
        tirmanma2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.75);
    }
    else
    {
        tirmanma.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        tirmanma2.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }

    //-------------------------------------------KAPAK-------------------------------------------
    if(js.GetRawButton(3))
    {
        kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.4);    //kapak açma 
    }
    else if(js.GetRawButton(4))
    {
        if(!limitSwitchmin.Get())          
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.07);
        else
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4);   //kapak kapama
    }
    else
    {
        if(limitSwitchmin.Get())
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.07);   
        else
            kapak.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }

    //-------------------------------------------INTEX-------------------------------------------
    if(js.GetRawAxis(3) == 1)
        intex.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
    else if(js.GetRawAxis(2) == 1)
        intex.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    else
        intex.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

    std::cout<< irSensor.Get()<< std::endl;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
