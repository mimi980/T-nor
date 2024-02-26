// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <ctre/Phoenix/motorcontrol/can/TalonFX.h>
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "Pid.h"
#include "frc/Encoder.h"
#include <frc/DigitalInput.h>
#include "NLCsv.h"
#include "RblUtils.h"

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  ctre::phoenix::motorcontrol::can::TalonFX m_MotorRight{1};
  ctre::phoenix::motorcontrol::can::TalonFX m_MotorLeft{2};
  ctre::phoenix::motorcontrol::can::TalonFX m_FeederMotor{3};

  rev::CANSparkMax m_Pivot{4, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  Pid m_pid{0.0, 0.0, 0.0, 0.0};
  frc::Encoder m_encoder{0, 1};

  frc::Joystick m_Jostick_Left{0};
  frc::Joystick m_Jostick_Right{1};

  frc::DigitalInput m_infraSensor{0};

  double m_position;
  double m_speed;
  double m_current;
  double m_speedShootHigh;
  double m_speedShoot;
  double m_speedAspiration;
  double m_speedCatch;
  double m_coeff;
  int m_count;

  enum State
  {
    Catch,
    Acceleration,
    Shoot,
    End
  };

  State m_state;

  NLCSV m_csv{3};
};
