// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/TakeNote.h"

TakeNote::TakeNote(Feeder *pFeeder, Intake *pIntake) : m_pFeeder(pFeeder), m_pIntake(pIntake)
{
  AddRequirements({m_pFeeder, m_pIntake});
}

// Called when the command is initially scheduled.
void TakeNote::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TakeNote::Execute() {}

// Called once the command ends or is interrupted.
void TakeNote::End(bool interrupted) {}

// Returns true when the command should end.
bool TakeNote::IsFinished()
{
  return false;
}
