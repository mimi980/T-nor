// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystem/Intake.h"
#include "subsystem/Feeder.h"

class TakeNote
    : public frc2::CommandHelper<frc2::Command, TakeNote>
{
public:
  TakeNote(Feeder *pFeeder, Intake *pIntake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Feeder *m_pFeeder;
  Intake *m_pIntake;
};
