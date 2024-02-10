// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <ctre/Phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include "frc/smartdashboard/SmartDashboard.h"

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

  enum class State 
  { 
    off,
    take,
    detected_forward,
    not_detected_backward,
    detected_stop,
    wait,
    stop,
    shoot
  };

  State m_state;

    frc::Joystick m_joystickRight{0};
    frc::Joystick m_joystickLeft{1};

    frc::DigitalInput m_infraSensor{0};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_motor775{1};

  double m_coeff;
  double m_timer;
  

};
