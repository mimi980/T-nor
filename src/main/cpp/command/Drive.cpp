// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/Drive.h"
#include <iostream>

Drive::Drive(std::function<double()> forward, std::function<double()> turn, Drivetrain *pDrivetrain, Camera *pCamera)
    : m_Forward(forward), m_Turn(turn), m_pDrivetrain(pDrivetrain), m_pCamera(pCamera)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({pDrivetrain, pCamera});
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute()
{
  double forward = m_Forward();
  double turn = m_Turn();
  if (m_pDrivetrain->drive_auto)
  {
    m_pCamera->SetSetpoint(0.0);
    std::cout << m_pCamera->m_output << "output" << std::endl;
    m_pDrivetrain->DriveAuto(forward, m_pCamera->m_output, m_pCamera->m_basePid.m_error);
  }
  else
  {
    m_pDrivetrain->Drive(forward, turn, false);
  }
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished()
{
  return false;
}
