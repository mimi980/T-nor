

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
    // m_climber.SetDefaultCommand(Climb([=]
    //                                   { return m_joystickLeft.GetY(); },
    //                                   &m_climber, &m_planetary));
}

void RobotContainer::ConfigureButtonBindings()
{

    m_buttonPreShoot.OnTrue(PreShoot(&m_shooter, &m_planetary, &m_camera).ToPtr());

    m_buttonShoot.OnTrue(Shoot(&m_shooter, &m_feeder, &m_planetary, &m_camera).ToPtr());

    m_buttonNearShoot.OnTrue(NearShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonAmpShoot.OnTrue(AmpShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonStageShoot.OnTrue(StageShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonTakeNote.OnTrue(TakeNote(&m_feeder, &m_intake, &m_planetary).ToPtr());

    m_buttonSpitNote.OnTrue(SpitNote(&m_feeder, &m_intake).ToPtr());
}