#include "command/Shoot.h"

Shoot::Shoot(Shooter *pShooter, Feeder *pFeeder, Planetary *pPlanetary, Camera *pCamera) : m_pShooter(pShooter), m_pFeeder(pFeeder), m_pPlanetary(pPlanetary), m_pCamera(pCamera)
{
  AddRequirements({m_pShooter, m_pFeeder, m_pPlanetary, m_pCamera});
}

void Shoot::Initialize()
{
  m_count = 0;
}

void Shoot::Execute()
{
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
    m_pShooter->SetShooter(SHOOTER_SPEED);
    if (m_pShooter->GetShooterVelocity() > GOALS_SHOOTER_SPEED)
    {
      m_state = State::Shoot;
    }
    break;
  case State::Shoot:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(SHOOTER_SPEED);
    if (!m_pFeeder->GetFeederInfraSensorValue())
    {
      m_state = State::Shooting;
      m_count = 0;
    }
    break;
  case State::Shooting:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(SHOOTER_SPEED);
    if (m_pFeeder->GetFeederInfraSensorValue() && m_count > 30)
    {
      m_state = State::End;
    }
    break;
  case State::End:
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    m_pShooter->SetShooter(STOP_SHOOTER_SPEED);
    break;

  default:
    break;
  }
}

void Shoot::End(bool interrupted)
{
  m_pFeeder->IsNoteLoaded = false;
}

bool Shoot::IsFinished()
{
  return false;
}
