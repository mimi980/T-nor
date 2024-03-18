// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "photon/PhotonCamera.h"
#include "photon/PhotonUtils.h"
#include "frc/filter/MedianFilter.h"
#include "frc/filter/LinearFilter.h"
#include "lib/RblUtils.h"
#include "lib/Pid.h"
#include "Constants.h"
#include <wpi/SpanExtras.h>
#include "photon/targeting/PhotonTrackedTarget.h"

class Camera : public frc2::SubsystemBase
{
public:
  Camera();
  int getAprilId();
  bool isAprilTagMode();
  double GetAngle();
  double GetOutput();
  void SetSetpoint(double setpoint);
  void Periodic() override;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double Air;
  double lastAir;
  double diffAir;

  Pid m_basePid{0.0, 0.1, 0.0, 0.0};
  double m_setpoint;
  double m_output;

  photon::PhotonCamera m_camera{"IRcam"};
  photon::PhotonPipelineResult result = m_camera.GetLatestResult();
  // std::span<photon::PhotonTrackedTarget> targetsList = result.GetTargets();

  frc::MedianFilter<double> m_verticalMedian = frc::MedianFilter<double>(3);
  frc::LinearFilter<double> m_horizontalErrorMovingAverage = frc::LinearFilter<double>::MovingAverage(3);

  // units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
  //     CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
  //     units::radian_t{m_camera.GetLatestResult().GetBestTarget().GetPitch()});
};
