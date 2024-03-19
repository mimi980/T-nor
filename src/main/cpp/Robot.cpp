// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::TakeNoteSwitch()
{
  m_robotContainer.m_planetary.SetSetpoint(TAKE_ANGLE);
  switch (m_stateTakeNote)
  {
  case StateTakeNote::Catch:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_intake.SetIntake(INTAKE_SPEED);
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_robotContainer.m_intake.SetIntake(STOP_INTAKE_SPEED);
      m_robotContainer.m_feeder.SetFeeder(SPIT_FEEDER_SPEED);
      m_stateTakeNote = StateTakeNote::Recul;
    }
    break;
  case StateTakeNote::Recul:
    m_robotContainer.m_feeder.SetFeeder(SPIT_FEEDER_SPEED);
    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
      m_stateTakeNote = StateTakeNote::Loaded;
    }
    break;
  case StateTakeNote::Loaded:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_feeder.IsNoteLoaded = true;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_takeNote = false;
    break;
  case StateTakeNote::End:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_intake.SetIntake(STOP_INTAKE_SPEED);
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    break;
  default:
    break;
  }
}

void Robot::ShootSwitch()
{
  shooter_speed = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetYaw(4))][2];
  planteray_angle = m_robotContainer.m_shooter.shooterDataTable[m_robotContainer.m_shooter.getNearestElementId(m_robotContainer.m_camera.GetYaw(4))][1];

  m_robotContainer.m_planetary.SetSetpoint(planteray_angle);
  m_goal = shooter_speed * 6379 * 0.90 * (10.0 / 12.0);
  m_count++;
  switch (m_stateShoot)
  {
  case StateShoot::Loaded:
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    if (m_robotContainer.m_feeder.IsNoteLoaded)
    {
      m_stateShoot = StateShoot::PreShoot;
    }
    break;
  case StateShoot::PreShoot:
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    if (NABS(m_robotContainer.m_shooter.GetShooterVelocity()) > m_goal && m_robotContainer.m_planetary.m_planetaryPid.AtSetpoint())
    {
      m_stateShoot = StateShoot::Shoot;
    }
    break;
  case StateShoot::Shoot:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    if (!m_robotContainer.m_feeder.GetFeederInfraSensorValue())
    {
      m_stateShoot = StateShoot::Shooting;
      m_count = 0;
    }
    break;
  case StateShoot::Shooting:
    m_robotContainer.m_feeder.SetFeeder(CATCH_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(shooter_speed);
    m_robotContainer.m_shooter.IsShoot = false;
    if (m_robotContainer.m_feeder.GetFeederInfraSensorValue() && m_count > 30)
    {
      m_stateShoot = StateShoot::End;
    }
    break;
  case StateShoot::End:
    m_robotContainer.m_feeder.IsNoteLoaded = false;
    m_robotContainer.m_planetary.SetSetpoint(REST_ANGLE);
    m_robotContainer.m_feeder.SetFeeder(STOP_FEEDER_SPEED);
    m_robotContainer.m_shooter.SetShooter(STOP_SHOOTER_SPEED);
    m_shoot = false;
    break;

  default:
    break;
  }
}

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

    m_follower.estimate(m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance(), m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance(), NDEGtoRAD(m_gyro.GetAngle()));
    m_follower.updateTarget(&m_TrajectoryPack, 0.02f);
    pout = m_follower.compute();
    m_robotContainer.m_drivetrain.DriveAuto(m_CrtzR.getVoltage(pout->m_rightVelocity, pout->m_rightAcceleration), m_CrtzL.getVoltage(pout->m_leftVelocity, pout->m_leftAcceleration), 0.0);
    std::cout << "pathFollowing" << std::endl;
    while (m_follower.getMessage(&message))
    {
      switch (message.m_id)
      {
      case TAKE_NOTE:
        m_takeNote = true;
        break;
      case SHOOT:
        m_shoot = true;

      default:
        break;
      }
    }

    break;
  case Robot::STATE::PATH_END:
    std::cout << "pathEND" << std::endl;
    break;
  default:
    NErrorIf(1, NERROR_UNAUTHORIZED_CASE);
    break;
  }

  if (m_takeNote)
  {
    TakeNoteSwitch();
  }
  if (m_shoot)
  {
    ShootSwitch();
  }
}

void Robot::TeleopInit()
{
}
void Robot::TeleopPeriodic()
{
  frc::SmartDashboard::PutBoolean("Loaded", m_robotContainer.m_feeder.IsNoteLoaded);
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
