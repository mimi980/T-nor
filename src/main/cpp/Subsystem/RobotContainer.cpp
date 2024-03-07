

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/RobotContainer.h"

RobotContainer::RobotContainer()
{
    m_drivetrain.SetDefaultCommand(Drive([=]
                                         { return m_joystickRight.GetY(); },
                                         [=]
                                         { return m_joystickLeft.GetY(); },
                                         &m_drivetrain));
}

void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton m_buttonPreShoot = frc2::JoystickButton(&m_joystickRight, 2);
    m_buttonPreShoot.WhileTrue(PreShoot(&m_shooter, &m_planetary, &m_camera).ToPtr());
}