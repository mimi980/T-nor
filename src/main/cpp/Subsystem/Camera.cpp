// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystem/Camera.h"

Camera::Camera()
{
    m_basePid.SetGains(BASE_PID_P, BASE_PID_I, BASE_PID_D);
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

// double Camera::GetAngle()
// {
//     if (m_camera.HasTargets())
//     {
//         double targetPitch = m_camera.GetLatestResult().GetBestTarget().GetPitch() + 0.01 * diffAir;
//         m_verticalMedian.Calculate(targetPitch);
//     }
//     else
//     {
//         m_verticalMedian.Calculate(m_verticalMedian.LastValue());
//     }
//     return m_verticalMedian.LastValue();
// }

double Camera::GetPitch(int Id_1, int Id_2)
{
    std::cout << m_camera.GetCameraName() << std::endl;
    if (m_camera.HasTargets())
    {
        photon::PhotonPipelineResult result = m_camera.GetLatestResult();
        std::span<const photon::PhotonTrackedTarget> targetsList = result.GetTargets();
        std::vector<photon::PhotonTrackedTarget>::iterator it;
        photon::PhotonTrackedTarget target;

        for (int i = 0; i < result.targets.size(); i++)
        {
            target = result.targets[i];

            if (target.GetFiducialId() == Id_1 || target.GetFiducialId() == Id_2)
            {
                m_verticalRollingAverage.add(target.GetPitch());
                std::cout << target.GetFiducialId() << std::endl;
            }
        }
    }
    else
    {
        m_verticalRollingAverage.add(m_verticalRollingAverage.get());
    }
    return m_verticalRollingAverage.get();
}

double Camera::GetYaw(int Id)
{
    if (m_camera.HasTargets())
    {
        photon::PhotonPipelineResult result = m_camera.GetLatestResult();
        std::span<const photon::PhotonTrackedTarget> targetsList = result.GetTargets();
        std::vector<photon::PhotonTrackedTarget>::iterator it;
        photon::PhotonTrackedTarget target;
        for (int i = 0; i < result.targets.size(); i++)
        {
            target = result.targets[i];
            if (target.GetFiducialId() == Id)
            {
                yaw = target.GetYaw();
                m_horizontalErrorMovingAverage.Calculate(yaw);
                break;
            }
        }
    }
    else
    {
        m_horizontalErrorMovingAverage.Calculate(m_horizontalErrorMovingAverage.LastValue());
    }
    return m_horizontalErrorMovingAverage.LastValue();
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
}