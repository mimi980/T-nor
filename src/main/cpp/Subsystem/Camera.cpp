// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Camera.h"

Camera::Camera() = default;

// This method will be called once per scheduler run

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
    if (m_camera.HasTargets())
    {
        return (targetPitch);
    }
    else
    {
        return 0.0;
    }
}

double Camera::GetAngle()
{
    double targetYaw = m_camera.GetLatestResult().GetBestTarget().GetYaw();
    m_verticalMedian.Calculate(targetYaw);
    return m_verticalMedian.LastValue();
}