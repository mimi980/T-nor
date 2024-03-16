#include "command/Shoot.h"

Shoot::Shoot(Shooter *pShooter, Feeder *pFeeder, Planetary *pPlanetary, Camera *pCamera) : m_pShooter(pShooter), m_pFeeder(pFeeder), m_pPlanetary(pPlanetary), m_pCamera(pCamera)
{
  AddRequirements({m_pShooter, m_pFeeder, m_pPlanetary, m_pCamera});
}

void Shoot::Initialize()
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

void Shoot::Execute()
{
  shooter_speed = m_pShooter->shooterDataTable[m_pShooter->getNearestElementId(m_pCamera->GetDistance())][2];
  planteray_angle = m_pShooter->shooterDataTable[m_pShooter->getNearestElementId(m_pCamera->GetDistance())][1];
  m_pPlanetary->SetSetpoint(planteray_angle);
  m_goal = shooter_speed * 6379 * 0.90 * (10.0 / 12.0);
  std::cout << shooter_speed << std::endl;
  std::cout << planteray_angle << std::endl;
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
    m_pShooter->SetShooter(shooter_speed);
    if (NABS(m_pShooter->GetShooterVelocity()) > m_goal)
    {
      m_state = State::Shoot;
    }
    break;
  case State::Shoot:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(shooter_speed);
    if (!m_pFeeder->GetFeederInfraSensorValue())
    {
      m_state = State::Shooting;
      m_count = 0;
    }
    break;
  case State::Shooting:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pShooter->SetShooter(shooter_speed);
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
void Shoot::End(bool interrupted)
{
  m_pFeeder->IsNoteLoaded = false;
}

bool Shoot::IsFinished()
{
  return false;
}
