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

    m_shooterMotorLeft.SetInverted(SHOOTER_MOTOR_LEFT_INVERTED);
    m_shooterMotorRight.SetInverted(SHOOTER_MOTOR_RIGHT_INVERTED);
};

/**
 * Sets the speed of the shooter motors.
 *
 * @param speed The speed to set the shooter motors to.
 */
void Shooter::SetShooter(double speed)
{
    m_shooterMotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    m_shooterMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void Shooter::SetAmpShooter(double speed)
{
    m_shooterMotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_shooterMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

/**
 * Returns the velocity of the shooter.
 *
 * @return The velocity of the shooter in units per second.
 */
double Shooter::GetShooterVelocity()
{
    return (((m_shooterMotorLeft.GetSensorCollection().GetIntegratedSensorVelocity() * 600.0 / 2048.0) + (m_shooterMotorRight.GetSensorCollection().GetIntegratedSensorVelocity() * 600.0 / 2048.0)) / 2.0) * 1.5;
}

/**
 * Finds the index of the nearest element in the shooter data table to the given target value.
 *
 * @param target The target value to find the nearest element to.
 * @return The index of the nearest element in the shooter data table.
 */
int Shooter::getNearestElementId(double target)
{
    int left = 0;
    int right = SHOOTER_TABLE_SIZE - 1;
    int mid;
    while (left < right)
    {
        mid = (left + right) / 2;
        if (shooterDataTable[mid][0] < target)
        {
            left = mid + 1;
        }
        else
        {
            right = mid;
        }
    }

    return left;
}
