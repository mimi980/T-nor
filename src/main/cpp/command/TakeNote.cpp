// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/TakeNote.h"

TakeNote::TakeNote(Feeder *pFeeder, Intake *pIntake) : m_pFeeder(pFeeder), m_pIntake(pIntake)
{
  AddRequirements({m_pFeeder, m_pIntake});
}

// Called when the command is initially scheduled.
void TakeNote::Initialize()
{
  m_state = State::Catch;
}

// Called repeatedly when this Command is scheduled to run
void TakeNote::Execute()
{
  switch (m_state)
  {
  case State::Catch:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pIntake->SetIntake(INTAKE_SPEED);
    if (!m_pFeeder->GetFeederInfraSensorValue())
    {
      m_pIntake->SetIntake(STOP_INTAKE_SPEED);
      m_pFeeder->SetFeeder(EJECT_FEEDER_SPEED);
      m_state = State::Recul;
    }
    break;
  case State::Recul:
    m_pFeeder->SetFeeder(EJECT_FEEDER_SPEED);
    if (m_pFeeder->GetFeederInfraSensorValue())
    {
      m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
      m_state = State::Loaded;
    }
    break;
  case State::Loaded:
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    break;
  default:
    break;
  }
}

// Called once the command ends or is interrupted.
void TakeNote::End(bool interrupted)
{
  m_state = State::Loaded;
}

// Returns true when the command should end.
bool TakeNote::IsFinished()
{
  return false;
}
