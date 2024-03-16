// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "frc/filter/MedianFilter.h"
#include "lib/RblUtils.h"
#include "Constants.h"

class Camera : public frc2::SubsystemBase
{
public:
  Camera();
  int getAprilId();
  bool isAprilTagMode();
  double GetDistance();
  double GetAngle();
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  photon::PhotonCamera m_camera{"IRcam"};
  frc::MedianFilter<double> m_verticalMedian = frc::MedianFilter<double>(5);
  // units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
  //     CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
  //     units::radian_t{m_camera.GetLatestResult().GetBestTarget().GetPitch()});
};
