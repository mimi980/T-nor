// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Subsystem/Climber.h"

Climber::Climber()
{
    m_climberMotorLeft.SetInverted(CLIMBER_MOTOR_LEFT_INVERTED);
    m_climberMotorRight.SetInverted(CLIMBER_MOTOR_RIGHT_INVERTED);

    m_climberMotorLeft.ConfigFactoryDefault();
    m_climberMotorRight.ConfigFactoryDefault();

    m_climberMotorLeft.EnableVoltageCompensation(true);
    m_climberMotorRight.EnableVoltageCompensation(true);

    m_climberMotorLeft.ConfigVoltageCompSaturation(CLIMBER_VOLTAGE_COMPENSATION);
    m_climberMotorRight.ConfigVoltageCompSaturation(CLIMBER_VOLTAGE_COMPENSATION);

    m_climberMotorLeft.ConfigOpenloopRamp(CLIMBER_RAMP);
    m_climberMotorRight.ConfigOpenloopRamp(CLIMBER_RAMP);

    m_climberMotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, CLIMBER_CURRENT_LIMIT, CLIMBER_CURRENT_LIMIT, 0.0));
    m_climberMotorLeft.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, CLIMBER_CURRENT_LIMIT, CLIMBER_CURRENT_LIMIT, 0.0));

    m_climberMotorLeft.Follow(m_climberMotorRight);

    m_climberEncoder.Reset();
    m_climberEncoder.SetDistancePerPulse((1.0 / 2048.0) / (150.0 / 18.0) * 360.0);
};

void Climber::SetClimberMotor(double speed)
{
    m_climberMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void Climber::SetSetpoint(double setpoint)
{
    m_pidClimber.SetSetpoint(setpoint);
}

double Climber::GetEncoder()
{
    return m_climberEncoder.GetDistance();
}

void Climber::Periodic()
{
    SetClimberMotor(m_pidClimber.Calculate(GetEncoder()));
}
