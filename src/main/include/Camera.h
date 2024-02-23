// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/filter/LinearFilter.h>
#include <photon/PhotonCamera.h>
#include "photon/PhotonUtils.h"
#include <units/length.h>
#include "RblUtils.h"

#define CAMERA_HEIGHT 0.78
#define TARGET_HEIGHT 1.37
#define CAMERA_PITCH 0

class Camera : public frc2::SubsystemBase
{
public:
    Camera();
    void Periodic() override;
    int getAprilId();
    bool isAprilTagMode();
    double GetDistance();
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    photon::PhotonCamera m_camera{"IRcam"};
    // units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
    //     CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
    //     units::radian_t{m_camera.GetLatestResult().GetBestTarget().GetPitch()});
};
