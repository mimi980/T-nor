// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "command/PreShoot.h"

PreShoot::PreShoot(Shooter *pShooter, Planetary *pPlanetary, Camera *pCamera) : m_pShooter(pShooter), m_pPlanetary(pPlanetary), m_pCamera(pCamera)
{
  AddRequirements({m_pShooter, m_pPlanetary, m_pCamera});
}

// Called when the command is initially scheduled.
void PreShoot::Initialize()
{
  // m_pShooter->IsPreShoot ? m_pShooter->IsPreShoot = false : m_pShooter->IsPreShoot = true;
}

// Called repeatedly when this Command is scheduled to run
void PreShoot::Execute()
{
  if (m_pCamera->getAprilId() == 4 or m_pCamera->getAprilId() == 7)
  {
    shooter_speed = m_pShooter->shooterDataTable[m_pShooter->getNearestElementId(m_pCamera->GetPitch(4))][2];
    planteray_angle = m_pShooter->shooterDataTable[m_pShooter->getNearestElementId(m_pCamera->GetPitch(4))][1];
    m_pShooter->SetShooter(shooter_speed);
    m_pPlanetary->SetSetpoint(planteray_angle);
  }
  else
  {
    m_pShooter->SetShooter(m_pShooter->shooterDataTable[SHOOTER_TABLE_SIZE - 1][2]);
    m_pPlanetary->SetSetpoint(m_pShooter->shooterDataTable[SHOOTER_TABLE_SIZE - 1][1]);
  }
}

// Called once the command ends or is interrupted.
void PreShoot::End(bool interrupted)
{
  m_pShooter->SetShooter(STOP_SHOOTER_SPEED);
  m_pPlanetary->SetSetpoint(REST_ANGLE);
}

// Returns true when the command should end.
bool PreShoot::IsFinished()
{
  return false;
}
