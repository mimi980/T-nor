

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    ConfigureButtonBindings();

    // m_drivetrain.SetDefaultCommand(Drive([=]
    //                                      { return m_xboxControllerPilote.GetLeftY(); },
    //                                      [=]
    //                                      { return m_xboxControllerPilote.GetRightX(); },
    //                                      &m_drivetrain, &m_camera));
    // m_climber.SetDefaultCommand(Climb([=]
    //                                   { return m_joystickLeft.GetY(); },
    //                                   &m_climber, &m_planetary));
    m_compressor.EnableDigital();
}

void RobotContainer::ConfigureButtonBindings()
{
    m_buttonPreShoot.WhileTrue(PreShoot(&m_shooter, &m_planetary, &m_camera).ToPtr());

    m_buttonShoot.WhileTrue(Shoot(&m_shooter, &m_feeder, &m_planetary, &m_camera).ToPtr());

    m_buttonNearShoot.WhileTrue(NearShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonAmpShoot.WhileTrue(AmpShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonStageShoot.WhileTrue(StageShoot(&m_shooter, &m_planetary, &m_feeder).ToPtr());

    m_buttonTakeNote.WhileTrue(TakeNote(&m_feeder, &m_intake, &m_planetary).ToPtr());
}