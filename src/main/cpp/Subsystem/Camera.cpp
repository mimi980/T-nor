// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Camera.h"

Camera::Camera()
{
    m_basePid.SetGains(0.005, 0.0, 0.0);
};

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

double Camera::GetAngle()
{
    if (m_camera.HasTargets())
    {
        double targetPitch = m_camera.GetLatestResult().GetBestTarget().GetPitch() + 0.01 * diffAir;
        m_verticalMedian.Calculate(targetPitch);
    }
    else
    {
        m_verticalMedian.Calculate(m_verticalMedian.LastValue());
    }
    return m_verticalMedian.LastValue();
    m_camera.GetLatestResult().GetTargets();
}

void Camera::SetSetpoint(double setpoint)
{
    m_basePid.SetSetpoint(setpoint);
}

void Camera::Periodic()
{
    if (m_camera.HasTargets())
    {
        Air = m_camera.GetLatestResult().GetBestTarget().GetArea();
        m_output = m_basePid.Calculate(m_camera.GetLatestResult().GetBestTarget().GetYaw());
    }

    diffAir = Air - lastAir;
    lastAir = Air;
    // std::cout << m_camera.GetLatestResult().GetBestTarget().GetYaw() << "error" << std::endl;
}