// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  m_gyro.Reset();
  m_gyro.Calibrate();

  // ######## NLMOTOR_CHARACTERIZATION ########
  // NLCHARACTERIZATION_TABLE characterization_table(4);
  // characterization_table.importTxt("/home/lvuser/auto/characterization_MultiVarLinearRegression.txt");

  m_CrtzL.m_forwardKv = 2.6412869101570307;
  m_CrtzL.m_backwardKv = 2.6248036368134255;        // = m_kv[1]
  m_CrtzL.m_forwardKa = 0.41029013114711876;        // = m_ka[0]
  m_CrtzL.m_backwardKa = 0.3713930975997702;        // = m_ka[1]
  m_CrtzL.m_forwardIntercept = 0.39281383839084655; // = m_intercept[0]
  m_CrtzL.m_backwardIntercept = -0.47477756289709444;

  m_CrtzR.m_forwardKv = 2.643306157356795;
  m_CrtzR.m_backwardKv = 2.632469106333122;        // = m_kv[1]
  m_CrtzR.m_forwardKa = 0.36456683427794917;       // = m_ka[0]
  m_CrtzR.m_backwardKa = 0.3257845553734638;       // = m_ka[1]
  m_CrtzR.m_forwardIntercept = 0.4218986448873328; // = m_intercept[0]
  m_CrtzR.m_backwardIntercept = -0.49001485320659466;

  m_TrajectoryPack.load("/home/lvuser/auto/red_right_db2cub_tcub_lightX.trk");

  m_follower.initialize(&m_TrajectoryPack);
  m_state = Robot::STATE::PATH_FOLLOWING;
}
void Robot::AutonomousPeriodic() 
{
  NLRAMSETEOUTPUT output;
  NLFOLLOWER_TANK_OUTPUT *pout = nullptr;

  NLTRJ_POSTED_MESSAGE message; // Posted Message

  // switch (m_state)
  // {
  // case Robot::STATE::PATH_ERROR:
  //   break;

  // case Robot::STATE::PATH_FOLLOWING:
  //   // *****************************************************    'THE' METHOD(e)
  //   // A) Feed back:
  //   // avec les encodeurs on estime la position du robot:
  //   //			l = distance parcourue par la roue gauche depuis le dernier reset encodeur.
  //   //			r = distance parcourue par la roue droite depuis le dernier reset encodeur.
  //   //
  //   //			dl et dr = distances parcourues par les roues gauche et droite depuis le dernier call.
  //   //			(note dl/dt = vitesse roue gauche et dr/dt = vitesse roue droite  )
  //   //

  //   m_follower.estimate(m_robotContainer.m_drivetrain.m_EncoderLeft.GetDistance(), m_robotContainer.m_drivetrain.m_EncoderRight.GetDistance(), NDEGtoRAD(m_gyro.GetAngle()));
  //   m_follower.updateTarget(&m_TrajectoryPack, 0.02f);
  //   pout = m_follower.compute();
  //   m_robotContainer.m_drivetrain.SetVoltage(m_CrtzR.getVoltage(pout->m_rightVelocity, pout->m_rightAcceleration), m_CrtzL.getVoltage(pout->m_leftVelocity, pout->m_leftAcceleration));

  //   while (m_follower.getMessage(&message))
  //   {
  //     switch (message.m_id)
  //     {
  //     case OPEN_INTAKE:
  //       m_robotContainer.m_intake.Open();
  //       break;
  //     case CLOSE_INTAKE:
  //       m_robotContainer.m_intake.Close();
  //       break;
  //     case ACTIVATE_INTAKE_WHEELS_TAKE:
  //       m_robotContainer.m_intake.SetSpeed(0.6);
  //       break;
  //     case ACTIVATE_INTAKE_WHEELS_DROPOFF:
  //       m_robotContainer.m_intake.SetSpeed(-0.6);
  //       break;
  //     case DEACTIVATE_INTAKE_WHEELS:
  //       m_robotContainer.m_intake.SetSpeed(0.0);
  //       break;
  //     case ACTIVATE_CONVEYOR_TAKE:
  //       m_robotContainer.m_conveyor.SetSpeed(0.6);
  //       break;
  //     case ACTIVATE_CONVEYOR_DROPOFF:
  //       m_robotContainer.m_conveyor.SetSpeed(-0.6);
  //       break;
  //     case DEACTIVATE_CONVEYOR:
  //       m_robotContainer.m_conveyor.SetSpeed(0.0);
  //       break;
  //     case TURRET_0:
  //       m_robotContainer.m_turret.SetSetpoint(0.0);
  //       break;
  //     case TURRET_L90:
  //       m_robotContainer.m_turret.SetSetpoint(-90.0);
  //       break;
  //     case TURRET_R90:
  //       m_robotContainer.m_turret.SetSetpoint(90.0);
  //       break;
  //     case TURRET_L180:
  //       m_robotContainer.m_turret.SetSetpoint(-180.0);
  //       break;
  //     case TURRET_R180:
  //       m_robotContainer.m_turret.SetSetpoint(180.0);
  //       break;
  //     case GRIPPER_TAKE_CUBE:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_gripper.Take(0.6);
  //       break;
  //     case GRIPPER_TAKE_CONE:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_gripper.Take(0.6);
  //       break;
  //     case GRIPPER_DROP_CUBE_OFF:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_gripper.Spit();
  //       break;
  //     case GRIPPER_DROP_CONE_OFF:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_gripper.Spit();
  //       break;
  //     case ARM_TO_LEVEL_CONE_1:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_elevator.SetSetpoint(0.72);
  //       m_robotContainer.m_arm.SetSetpoint(NDEGtoRAD(105.0));
  //       m_robotContainer.m_gripper.DropMidleCone = true;
  //       break;
  //     case ARM_TO_LEVEL_CONE_2:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_elevator.SetSetpoint(0.97);
  //       m_robotContainer.m_arm.SetSetpoint(NDEGtoRAD(120.0));
  //       break;
  //     case ARM_TO_LEVEL_CUBE_1:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_elevator.SetSetpoint(0.43);
  //       m_robotContainer.m_arm.SetSetpoint(NDEGtoRAD(97.0));
  //       m_robotContainer.m_gripper.DropHighCube = true;
  //       break;
  //     case ARM_TO_LEVEL_CUBE_2:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_elevator.SetSetpoint(0.98);
  //       m_robotContainer.m_arm.SetSetpoint(NDEGtoRAD(115.0));
  //       m_robotContainer.m_gripper.DropHighCube = true;
  //       break;
  //     case ARM_HOME:
  //       m_takeCubeInRobot = false;
  //       m_robotContainer.m_elevator.SetSetpoint(0.0);
  //       m_robotContainer.m_arm.SetSetpoint(NDEGtoRAD(100.0));
  //       break;
  //     case TAKE_CUBE_IN_ROBOT:
  //       m_takeCubeInRobot = true;
  //       m_StateTakeCubeRobot = StateTakeCubeRobot::Init;
  //       break;
      
  //     default:
  //       break;
  //     }
  //   }

  //   break;

  // case Robot::STATE::PATH_END:
  //   m_autoCube = true;

  //   break;
  // default:
  //   NErrorIf(1, NERROR_UNAUTHORIZED_CASE);
  //   break;
  // }
  
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
