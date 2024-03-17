

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    ConfigureButtonBindings();

    m_drivetrain.SetDefaultCommand(Drive([=]
                                         { return m_joystickRight.GetY(); },
                                         [=]
                                         { return m_joystickLeft.GetZ(); },
                                         &m_drivetrain, &m_camera));
}

void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton m_buttonPreShoot = frc2::JoystickButton(&m_joystickRight, 2);
    m_buttonPreShoot.OnTrue(PreShoot(&m_shooter, &m_planetary, &m_camera).ToPtr());

    frc2::JoystickButton m_buttonShoot = frc2::JoystickButton(&m_joystickRight, 1);
    m_buttonShoot.OnTrue(Shoot(&m_shooter, &m_feeder, &m_planetary, &m_camera).ToPtr());

    frc2::JoystickButton m_buttonAmpShoot = frc2::JoystickButton(&m_joystickRight, 4);
    m_buttonAmpShoot.OnTrue(AmpShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    frc2::JoystickButton m_buttonNearShoot = frc2::JoystickButton(&m_joystickRight, 3);
    m_buttonNearShoot.OnTrue(NearShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    frc2::JoystickButton m_buttonStageShoot = frc2::JoystickButton(&m_joystickRight, 5);
    m_buttonStageShoot.OnTrue(StageShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    frc2::JoystickButton m_buttonIntake = frc2::JoystickButton(&m_joystickLeft, 1);
    m_buttonIntake.OnTrue(TakeNote(&m_feeder, &m_intake, &m_planetary).ToPtr());

    frc2::JoystickButton m_buttonSpit = frc2::JoystickButton(&m_joystickLeft, 10);
    m_buttonSpit.OnTrue(SpitNote(&m_feeder, &m_intake).ToPtr());
}