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
#include <vector>
#include "lib/NRollingAverage.h"

class Camera : public frc2::SubsystemBase
{
public:
  Camera();
  int getAprilId();
  double GetAngle();
  double GetPitch(int Id_1, int Id_2);
  double GetOutput();
  void SetSetpoint(double setpoint);
  double GetYaw(int Id);
  void Periodic() override;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  double Air;
  double lastAir;
  double diffAir;
  double yaw;

  Pid m_basePid{0.0, BASE_PID_P, BASE_PID_I, BASE_PID_D};
  double m_setpoint;
  double m_output;

  photon::PhotonCamera m_camera{"IRcam"};

  frc::LinearFilter<double> m_horizontalErrorMovingAverage = frc::LinearFilter<double>::MovingAverage(3);

  NdoubleRollingAverage m_verticalRollingAverage{10};

  // units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
  //     CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,
  //     units::radian_t{m_camera.GetLatestResult().GetBestTarget().GetPitch()});
};
