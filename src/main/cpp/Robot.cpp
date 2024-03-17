// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
  m_gyro.Reset();
  m_gyro.Calibrate();

  // ######## NLMOTOR_CHARACTERIZATION ########
  // NLCHARACTERIZATION_TABLE characterization_table(4);
  // characterization_table.importTxt("/home/lvuser/auto/characterization_MultiVarLinearRegression.txt");

  m_CrtzL.m_forwardKv = 2.6412869101570307f;
  m_CrtzL.m_backwardKv = 2.6248036368134255f;        // = m_kv[1]
  m_CrtzL.m_forwardKa = 0.41029013114711876f;        // = m_ka[0]
  m_CrtzL.m_backwardKa = 0.3713930975997702f;        // = m_ka[1]
  m_CrtzL.m_forwardIntercept = 0.39281383839084655f; // = m_intercept[0]
  m_CrtzL.m_backwardIntercept = -0.47477756289709444f;

  m_CrtzR.m_forwardKv = 2.643306157356795f;
  m_CrtzR.m_backwardKv = 2.632469106333122f;        // = m_kv[1]
  m_CrtzR.m_forwardKa = 0.36456683427794917f;       // = m_ka[0]
  m_CrtzR.m_backwardKa = 0.3257845553734638f;       // = m_ka[1]
  m_CrtzR.m_forwardIntercept = 0.4218986448873328f; // = m_intercept[0]
  m_CrtzR.m_backwardIntercept = -0.49001485320659466f;

  m_TrajectoryPack.load("/home/lvuser/auto/tout_droit.trk");

  m_follower.initialize(&m_TrajectoryPack);
  m_state = Robot::STATE::PATH_FOLLOWING;
}
void Robot::AutonomousPeriodic()
{
  NLRAMSETEOUTPUT output;
  NLFOLLOWER_TANK_OUTPUT *pout = nullptr;

  NLTRJ_POSTED_MESSAGE message; // Posted Message

  switch (m_state)
  {
  case Robot::STATE::PATH_ERROR:
    break;

  case Robot::STATE::PATH_FOLLOWING:
    // *****************************************************    'THE' METHOD(e)
    // A) Feed back:
    // avec les encodeurs on estime la position du robot:
    //			l = distance parcourue par la roue gauche depuis le dernier reset encodeur.
    //			r = distance parcourue par la roue droite depuis le dernier reset encodeur.
    //
    //			dl et dr = distances parcourues par les roues gauche et droite depuis le dernier call.
    //			(note dl/dt = vitesse roue gauche et dr/dt = vitesse roue droite  )
    //

    m_follower.estimate(m_drivetrain.m_EncoderLeft.GetDistance(), m_drivetrain.m_EncoderRight.GetDistance(), NDEGtoRAD(m_gyro.GetAngle()));
    m_follower.updateTarget(&m_TrajectoryPack, 0.02f);
    pout = m_follower.compute();
    m_drivetrain.DriveAuto(m_CrtzR.getVoltage(pout->m_rightVelocity, pout->m_rightAcceleration), m_CrtzL.getVoltage(pout->m_leftVelocity, pout->m_leftAcceleration));
    std::cout << "pathFollowing" << std::endl;
    break;

  case Robot::STATE::PATH_END:
    std::cout << "pathEND" << std::endl;
    break;
  default:
    NErrorIf(1, NERROR_UNAUTHORIZED_CASE);
    break;
  }
}

void Robot::TeleopInit()
{
}
void Robot::TeleopPeriodic()
{
  if (m_robotContainer.m_joystickLeft.GetRawButtonPressed(1))
  {
    m_robotContainer.m_intake.IsIntaked ? m_robotContainer.m_intake.IsIntaked = false : m_robotContainer.m_intake.IsIntaked = true;
  }
  if (m_robotContainer.m_joystickRight.GetRawButtonPressed(2))
  {
    m_robotContainer.m_shooter.IsPreShoot ? m_robotContainer.m_shooter.IsPreShoot = false : m_robotContainer.m_shooter.IsPreShoot = true;
  }

  if (m_robotContainer.m_joystickLeft.GetRawButton(2))
  {
    m_robotContainer.m_drivetrain.drive_auto = true;
  }
  else
  {
    m_robotContainer.m_drivetrain.drive_auto = false;
  }
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
