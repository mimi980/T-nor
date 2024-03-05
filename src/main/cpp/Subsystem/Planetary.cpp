// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Planetary.h"

Planetary::Planetary()
{
    // Initialize your motor controllers here
    m_planetaryMotor.RestoreFactoryDefaults();
    m_planetaryMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_planetaryMotor.SetInverted(false);
    m_planetaryMotor.SetOpenLoopRampRate(0.5);
    m_planetaryMotor.SetSmartCurrentLimit(40);
    m_planetaryMotor.EnableVoltageCompensation(12);
};

void Planetary::SetPlanetary(double speed)
{
    m_planetaryMotor.Set(speed);
}

void Planetary::SetSetpoint(double setpoint)
{
    m_planetaryPid.SetSetpoint(setpoint);
}
