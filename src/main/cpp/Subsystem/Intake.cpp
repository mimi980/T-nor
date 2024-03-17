// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Intake.h"

Intake::Intake()
{
    m_intakeMotorRight.ConfigFactoryDefault();
    m_intakeMotorRight.SetInverted(INTAKE_MOTOR_INVERTED);
    m_intakeMotorRight.EnableVoltageCompensation(true);
    m_intakeMotorRight.ConfigVoltageCompSaturation(INTAKE_VOLTAGE_COMPENSATION);
    m_intakeMotorRight.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_intakeMotorRight.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, INTAKE_CURRENT_LIMIT, INTAKE_CURRENT_LIMIT, 0));
    // m_intakeMotorRight.ConfigContinuousCurrentLimit(20);

    // m_intakeMotorLeft.ConfigFactoryDefault();
    // m_intakeMotorLeft.SetInverted(INTAKE_MOTOR_INVERTED_LEFT);
    // m_intakeMotorLeft.EnableVoltageCompensation(true);
    // m_intakeMotorLeft.ConfigVoltageCompSaturation(INTAKE_VOLTAGE_COMPENSATION);
    // m_intakeMotorLeft.EnableCurrentLimit(true);
    // m_intakeMotorLeft.ConfigContinuousCurrentLimit(20);
    IsIntaked = true;
};

void Intake::SetIntake(double speed)
{
    m_intakeMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    // m_intakeMotorLeft.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}
