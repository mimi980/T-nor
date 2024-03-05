// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Planetary.h"

Planetary::Planetary()
{
    m_planetaryMotor.RestoreFactoryDefaults();
    m_planetaryMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_planetaryMotor.SetInverted(PLANETARY_MOTOR_INVERTED);
    m_planetaryMotor.SetClosedLoopRampRate(PLANETARY_RAMP);
    m_planetaryMotor.SetSmartCurrentLimit(PLANETARY_CURRENT_LIMIT);
    m_planetaryMotor.EnableVoltageCompensation(PLANETARY_VOLTAGE_COMPENSATION);

    m_planetaryEncoder.Reset();
    m_planetaryEncoder.SetDistancePerPulse(PLANETARY_DISTANCE_PER_PULSE);

    m_planetaryPid.Reset();
    m_planetaryPid.SetGains(PLANETARY_PID_P, PLANETARY_PID_I, PLANETARY_PID_D);
    m_planetaryPid.SetSetpoint(0.0);
};

void Planetary::SetPlanetary(double speed)
{
    m_planetaryMotor.Set(speed);
}

void Planetary::SetSetpoint(double setpoint)
{
    m_planetaryPid.SetSetpoint(setpoint);
}

double Planetary::GetEncoder()
{
    return m_planetaryEncoder.GetDistance();
}

void Planetary::Periodic()
{
    SetPlanetary(m_planetaryPid.Calculate(GetEncoder()));
}