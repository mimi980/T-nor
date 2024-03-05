// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Feeder.h"

Feeder::Feeder() = default;

// This method will be called once per scheduler run

void Feeder::SetFeeder(double speed)
{
    m_feederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

bool Feeder::GetFeederInfraSensorValue()
{
    return m_feederInfraSensor.Get();
}