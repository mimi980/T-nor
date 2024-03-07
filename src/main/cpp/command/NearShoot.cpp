// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/NearShoot.h"

NearShoot::NearShoot(Shooter *pShooter, Planetary *pPlanetary, Feeder *pFeeder)
    : m_pShooter(pShooter), m_pPlanetary(pPlanetary), m_pFeeder(pFeeder)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({pShooter, pPlanetary, pFeeder});
}

// Called when the command is initially scheduled.
void NearShoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void NearShoot::Execute() {}

// Called once the command ends or is interrupted.
void NearShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool NearShoot::IsFinished()
{
  return false;
}
