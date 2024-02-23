// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Camera.h"
#include <iostream>

Camera::Camera() = default;

// This method will be called once per scheduler run
void Camera::Periodic() {}

int Camera::getAprilId()
{

    if (m_camera.HasTargets())
    {
        return m_camera.GetLatestResult().GetBestTarget().GetFiducialId();
    }
    else
    {
        return 0;
    }
}
bool Camera::isAprilTagMode()
{
    return !m_camera.GetDriverMode() && (m_camera.GetPipelineIndex() == 0);
}

double Camera::GetDistance()
{
    double targetPitch = m_camera.GetLatestResult().GetBestTarget().GetPitch();
    std::cout << targetPitch << std::endl;
    return (TARGET_HEIGHT - CAMERA_HEIGHT) / units::math::tan(units::radian_t(NDEGtoRAD(targetPitch + CAMERA_PITCH)));
}
