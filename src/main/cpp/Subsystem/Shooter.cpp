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

    m_shooterMotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, SHOOTER_CURRENT_LIMIT, SHOOTER_CURRENT_LIMIT, 0));
    m_shooterMotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, SHOOTER_CURRENT_LIMIT, SHOOTER_CURRENT_LIMIT, 0));

    m_shooterMotorLeft.ConfigClosedloopRamp(SHOOTER_RAMP);
    m_shooterMotorRight.ConfigClosedloopRamp(SHOOTER_RAMP);

    m_shooterMotorLeft.EnableVoltageCompensation(true);
    m_shooterMotorRight.EnableVoltageCompensation(true);

    m_shooterMotorLeft.ConfigVoltageCompSaturation(SHOOTER_VOLTAGE_COMPENSATION);
    m_shooterMotorRight.ConfigVoltageCompSaturation(SHOOTER_VOLTAGE_COMPENSATION);

    m_shooterMotorLeft.SetInverted(false);
    m_shooterMotorRight.SetInverted(true);
};

void Shooter::SetShooter(double speed)
{
    m_shooterMotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    m_shooterMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

double Shooter::GetShooterVelocity()
{
    return (((m_shooterMotorLeft.GetSensorCollection().GetIntegratedSensorVelocity() * 600.0 / 2048.0) + (m_shooterMotorRight.GetSensorCollection().GetIntegratedSensorVelocity() * 600.0 / 2048.0)) / 2.0) * 1.5;
}
