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
void NearShoot::Initialize()
{
  m_count = 0;
  // m_pShooter->IsShoot ? m_pShooter->IsShoot = false : m_pShooter->IsShoot = true;
  m_pShooter->IsPreShoot = false;
  // if (!m_pShooter->IsShoot)
  // {
  //   m_state = State::End;
  // }
  // else
  // {
  //   m_state = State::Loaded;
  // }
  m_state = State::Loaded;
}

// Called repeatedly when this Command is scheduled to run
void NearShoot::Execute()
{
  m_pPlanetary->SetSetpoint(38.0);
  m_count++;
  switch (m_state)
  {
  case State::Loaded:
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    if (m_pFeeder->IsNoteLoaded)
    {
      m_state = State::PreShoot;
    }
    break;
  case State::PreShoot:
    m_pShooter->SetShooter(0.5);
    if (NABS(m_pShooter->GetShooterVelocity()) > 3000)
    {
      m_state = State::Shoot;
    }
    break;
  case State::Shoot:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(0.5);
    if (!m_pFeeder->GetFeederInfraSensorValue())
    {
      m_state = State::Shooting;
      m_count = 0;
    }
    break;
  case State::Shooting:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(0.5);
    m_pShooter->IsShoot = false;
    if (m_pFeeder->GetFeederInfraSensorValue() && m_count > 30)
    {
      m_state = State::End;
    }
    break;
  case State::End:
    m_pPlanetary->SetSetpoint(0.0);
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    m_pShooter->SetShooter(STOP_SHOOTER_SPEED);
    break;

  default:
    break;
  }
}

// Called once the command ends or is interrupted.
void NearShoot::End(bool interrupted)
{
  m_pFeeder->IsNoteLoaded = false;
}

// Returns true when the command should end.
bool NearShoot::IsFinished()
{
  return false;
}
