#include "command/TakeNote.h"

TakeNote::TakeNote(Feeder *pFeeder, Intake *pIntake, Planetary *pPlanetary) : m_pFeeder(pFeeder), m_pIntake(pIntake), m_pPlanetary(pPlanetary)
{
  AddRequirements({m_pFeeder, m_pIntake, m_pPlanetary});
}

void TakeNote::Initialize()
{
  if (!m_pIntake->IsIntaked)
  {
    m_state = State::End;
  }
  else
  {
    m_state = State::Catch;
  }
  m_pPlanetary->SetSetpoint(0.0);
}

void TakeNote::Execute()
{
  switch (m_state)
  {
  case State::Catch:
    m_pFeeder->SetFeeder(CATCH_FEEDER_SPEED);
    m_pIntake->SetIntake(INTAKE_SPEED);
    std::cout << m_pFeeder->GetFeederInfraSensorValue() << std::endl;
    if (!m_pFeeder->GetFeederInfraSensorValue())
    {
      m_pIntake->SetIntake(STOP_INTAKE_SPEED);
      m_pFeeder->SetFeeder(SPIT_FEEDER_SPEED);
      m_state = State::Recul;
    }
    break;
  case State::Recul:
    m_pFeeder->SetFeeder(SPIT_FEEDER_SPEED);
    if (m_pFeeder->GetFeederInfraSensorValue())
    {
      m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
      m_state = State::Loaded;
    }
    break;
  case State::Loaded:
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    m_pFeeder->IsNoteLoaded = true;
    m_pIntake->IsIntaked = false;
    break;
  case State::End:
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    m_pIntake->SetIntake(STOP_INTAKE_SPEED);
    m_pFeeder->IsNoteLoaded = false;
    break;
  default:
    break;
  }
}

void TakeNote::End(bool interrupted)
{
  m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
  m_pIntake->SetIntake(STOP_INTAKE_SPEED);
}

bool TakeNote::IsFinished()
{
  if (m_pFeeder->IsNoteLoaded)
  {
    m_pFeeder->SetFeeder(STOP_FEEDER_SPEED);
    m_pIntake->SetIntake(STOP_INTAKE_SPEED);
    return true;
  }
  else if (!m_pIntake->IsIntaked)
  {
    return true;
  }
  else
  {
    return false;
  }
}
