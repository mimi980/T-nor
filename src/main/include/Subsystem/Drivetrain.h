#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <frc/Doublesolenoid.h>
#include "rev/CANSparkMax.h"
#include <lib/NRollingAverage.h>
#include <lib/rate_limiter.h>
#include <lib/Dynamic.h>
#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "lib/RblUtils.h"
#include "lib/utils.h"
#include <math.h>

#define MOTOR_WF_RPM 5874.0 // Free Speed théorique du moteur à la tension de reference (12V)
#define MOTOR_TS_NM 3.35    // Stall Torque théorique du moteur à la tension de reference (12V)

#define REDUC_V1 9.88
#define REDUC_V2 5.98

#define TURNING_TOLERANCE 0.05

#define UP_SHIFTING_POINT_JOYSTICK_V 0.8                                    // Valeur minimum du joystick V pour passer en vitesse 2
#define UP_SHIFTING_POINT_JOYSTICK_V_VARIATION 0.0                          // Valeur minimum de la variation (=delta) du joystick V pour passer en vitesse 2
#define UP_SHIFTING_POINT_GEARBOXES_OUT_RPM (MOTOR_WF_RPM * 0.5 / REDUC_V1) // Valeur minimum de la vitesse de sortie de boites pour passer en vitesse 2
#define UP_SHIFTING_POINT_GEARBOXES_OUT_RPM2 0.0                            // Valeur minimum de l'accel.  de sortie de boites pour passer en vitesse 2

#define KICKDOWN_SHIFTING_POINT_GEARBOXES_OUT_RPM (MOTOR_WF_RPM * 0.3 / REDUC_V2) // Valeur max "haute" de la vitesse de sortie de boites pour retrograder en vitesse 1
#define KICKDOWN_SHIFTING_POINT_JOYSTICK_V 0.6                                    // Valeur minimum du joystick V pour retrograder en vitesse 1 afin de re-accelerer fort
#define KICKDOWN_SHIFTING_POINT_JOYSTICK_V_VARIATION 0.2                          // Valeur minimum de la variation (=delta) du joystick V pour retrograder en vitesse 1

#define COASTDOWN_SHIFTING_POINT_GEARBOXES_OUT_RPM (MOTOR_WF_RPM * 0.05 / REDUC_V2) // Valeur max "basse" de la vitesse de sortie de boites pour retrograder en vitesse 1

#define GEARSHIFTING_TIMELOCK 0.26 // 0.08

#define TICK_DT 0.02             // durée d'un tick en seconde
#define AVERAGE_SAMPLES_NUMBER 5 // nombre de samples pour la moyenne

class Drivetrain : public frc2::SubsystemBase
{
public:
  Drivetrain();
  void Set(double speed); // faire autre chose y a moyen

  void ActiveBallShifterV1(); // ok
  void ActiveBallShifterV2();
  void ChangeBallShifter();                                                // ok
  void Drive(double joystickLeft, double joystickRight, bool brakeButton); // ok

  void DriveAuto(double speed, double rotation);
  double Calcul_De_Notre_Brave_JM(double forward, double turn, bool wheelSide); // Si wheelSide 0: roue droite / Si wheelSide 1: roue gauche

  bool isUpshiftingAllowed();
  bool isKickdownShiftingAllowed(); // ok

  void ShiftGearUp();
  void ShiftGearDown();
  void Reset();
  void SetVoltage(double right, double left);

  double getLeftDistance();
  double getRightDistance();

  // Côté gauche
  Dynamic m_GearboxLeftOutRawRpt;                    // Vitesse instantanée de sortie de boite ( mesurée par le TroughBore Encoder )
  NdoubleRollingAverage m_GearboxLeftOutAveragedRpt; // Vitesse moyenne de sortie de boite (Moyenne glissante)
  double m_GearboxLeftOutAdjustedRpm;                // Vitesse de la gearbox gauche en RPM ( combinaison linéaire de la vitesse moyenne de sortie de boite et de la vitesse moyenne du supermoteur en entrée de boite)

  // Côté droit
  Dynamic m_GearboxRightOutRawRpt;                    // Vitesse instantanée de sortie de boite ( mesurée par le TroughBore Encoder )
  NdoubleRollingAverage m_GearboxRightOutAveragedRpt; // Vitesse moyenne de sortie de boite (Moyenne glissante)
  double m_GearboxRightOutAdjustedRpm;                // Vitesse de la gearbox droite en RPM ( combinaison linéaire de la vitesse moyenne de sortie de boite et de la vitesse moyenne du supermoteur en entrée de boite)

  Dynamic m_GearboxesOutAdjustedRpm;                            // Vitesse (translation) du robot exprimée en RPM
  NdoubleRollingAverage m_GearboxesOutAveragedAccelerationRpm2; // Acceleration moyennée (translation) du robot exprimée RPM²

  double m_GearShiftingTimeLock; // Temps de blocage du changement de vitesse
  double m_CurrentGearboxRatio;  // Rapport (Reduction) de la vitesse engagée dans la  boite (V1 ou V2)

  Dynamic m_JoystickRaw_V;
  RateLimiter m_JoystickPrelimited_V; // joystick V rate limiter 1
  RateLimiter m_JoystickLimited_V;    // joystick V rate limiter 2

  Dynamic m_JoystickRaw_W;
  RateLimiter m_JoystickPrelimited_W; // joystick W rate limiter 1
  RateLimiter m_JoystickLimited_W;    // joystick W rate limiter 2

  enum class State
  {
    lowGear,
    highGear
  };

  State m_State;

  double m_sigma;

  // bool drive_auto = false;

  frc::Encoder m_EncoderLeft{ID_ENCODER_DRIVE_TRAIN_LEFT_A, ID_ENCODER_DRIVE_TRAIN_LEFT_B, false, frc::Encoder::k4X};
  frc::Encoder m_EncoderRight{ID_ENCODER_DRIVE_TRAIN_RIGHT_A, ID_ENCODER_DRIVE_TRAIN_RIGHT_B, true, frc::Encoder::k4X};

private:
  rev::CANSparkMax m_MotorRight1{ID_MOTOR_DRIVE_TRAIN_RIGHT, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_MotorRight2{ID_MOTOR_DRIVE_TRAIN_RIGHT_2, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_MotorLeft1{ID_MOTOR_DRIVE_TRAIN_LEFT, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_MotorLeft2{ID_MOTOR_DRIVE_TRAIN_LEFT_2, rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DoubleSolenoid m_BallShifterSolenoid{frc::PneumaticsModuleType::CTREPCM, ID_SOLENOID_SHIFTER_A, ID_SOLENOID_SHIFTER_B};
};