// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Subsystem/Climber.h"
#include "Subsystem/Planetary.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Climb
    : public frc2::CommandHelper<frc2::Command, Climb>
{
public:
  Climb(Climber *pClimber, Planetary *pPlanetary);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Climber *m_pClimber;
  Planetary *m_pPlanetary;
};
