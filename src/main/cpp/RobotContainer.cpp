// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer()
{
    ConfigureButtonBindings();

    m_CameraPilote = frc::CameraServer::StartAutomaticCapture();
    m_CameraPilote.SetResolution(320, 240);
    m_CameraPilote.SetFPS(12);

    m_compressor.EnableDigital();
};

// ################### COMMANDS ###################

void RobotContainer::ConfigureButtonBindings(){}
