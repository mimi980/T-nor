// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/Climb.h"

Climb::Climb(std::function<double()> forward, Climber *pClimber, Planetary *pPlanetary) : m_Forward(forward), m_pClimber(pClimber), m_pPlanetary(pPlanetary)
{
  AddRequirements({pClimber, pPlanetary});
}

// Called when the command is initially scheduled.
void Climb::Initialize()
{
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute()
{
  double forward = m_Forward();
  m_pClimber->SetSetpoint(50 * forward);
  m_pPlanetary->SetSetpoint(80);
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {}

// Returns true when the command should end.
bool Climb::IsFinished()
{
  return false;
}
