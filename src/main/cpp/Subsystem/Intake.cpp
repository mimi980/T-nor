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
};

void Intake::SetIntake(double speed)
{
    m_intakeMotorRight.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}
