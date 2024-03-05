// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Shooter.h"

Shooter::Shooter()
{
    m_shooterMotorLeft.ConfigFactoryDefault();
    m_shooterMotorRight.ConfigFactoryDefault();

    m_shooterMotorLeft.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_shooterMotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_shooterMotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    m_shooterMotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 40, 40, 0));

    m_shooterMotorLeft.ConfigClosedloopRamp(0.5);
    m_shooterMotorRight.ConfigClosedloopRamp(0.5);

    m_shooterMotorLeft.EnableVoltageCompensation(true);
    m_shooterMotorRight.EnableVoltageCompensation(true);

    m_shooterMotorLeft.ConfigVoltageCompSaturation(12);
    m_shooterMotorRight.ConfigVoltageCompSaturation(12);

    m_shooterMotorLeft.SetInverted(false);
    m_shooterMotorRight.SetInverted(true);
};

// This method will be called once per scheduler run
void Shooter::Periodic() {}
