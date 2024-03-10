// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/SpitNote.h"

SpitNote::SpitNote(Feeder *feeder, Intake *intake) : m_feeder(feeder), m_intake(intake)
{
  AddRequirements({feeder, intake});
}

// Called when the command is initially scheduled.
void SpitNote::Initialize()
{
  m_count = 0;
}

// Called repeatedly when this Command is scheduled to run
void SpitNote::Execute()
{
  m_feeder->SetFeeder(SPIT_FEEDER_SPEED);
  m_intake->SetIntake(SPIT_INTAKE_SPEED);
}

// Called once the command ends or is interrupted.
void SpitNote::End(bool interrupted)
{
  m_feeder->IsNoteLoaded = false;
  m_feeder->SetFeeder(STOP_FEEDER_SPEED);
  m_intake->SetIntake(STOP_INTAKE_SPEED);
}

// Returns true when the command should end.
bool SpitNote::IsFinished()
{
  if (m_count > 50)
  {
    m_count = 0;
    return true;
  }
  else
  {
    m_count++;
    return false;
  }
}
