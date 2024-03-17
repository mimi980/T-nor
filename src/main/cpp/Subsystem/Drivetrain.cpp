#include "subsystem/Drivetrain.h"

Drivetrain::Drivetrain() : m_GearboxLeftOutAveragedRpt(AVERAGE_SAMPLES_NUMBER),
                           m_GearboxRightOutAveragedRpt(AVERAGE_SAMPLES_NUMBER),
                           m_GearboxesOutAveragedAccelerationRpm2(AVERAGE_SAMPLES_NUMBER)

{
    m_MotorLeft1.RestoreFactoryDefaults(); // reset des paramètres du moteur
    m_MotorLeft2.RestoreFactoryDefaults();

    m_MotorRight1.RestoreFactoryDefaults();
    m_MotorRight2.RestoreFactoryDefaults();

    m_MotorLeft1.SetSmartCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
    m_MotorLeft2.SetSmartCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);

    m_MotorRight1.SetSmartCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);
    m_MotorRight2.SetSmartCurrentLimit(DRIVETRAIN_CURRENT_LIMIT);

    m_MotorLeft1.SetInverted(DRIVETRAIN_MOTOR_LEFT_INVERTED); // inversion des moteurs
    m_MotorLeft2.SetInverted(DRIVETRAIN_MOTOR_LEFT_INVERTED);

    m_MotorRight1.SetInverted(DRIVETRAIN_MOTOR_RIGHT_INVERTED);
    m_MotorRight2.SetInverted(DRIVETRAIN_MOTOR_RIGHT_INVERTED);

    m_MotorLeft1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_MotorLeft2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_MotorRight1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_MotorRight2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_MotorLeft1.EnableVoltageCompensation(DRIVETRAIN_VOLTAGE_COMPENSATION);
    m_MotorLeft2.EnableVoltageCompensation(DRIVETRAIN_VOLTAGE_COMPENSATION);

    m_MotorRight1.EnableVoltageCompensation(DRIVETRAIN_VOLTAGE_COMPENSATION);
    m_MotorRight2.EnableVoltageCompensation(DRIVETRAIN_VOLTAGE_COMPENSATION);

    m_MotorLeft2.Follow(m_MotorLeft1); // init follower moteurs
    m_MotorRight2.Follow(m_MotorRight1);

    /*m_JoystickPrelimited_V.Reset(0.0, 0.0, 2.0); // reset des rate limiters
    m_JoystickLimited_V.Reset(0.0, 0.0, 0.035);  // 0.04

    m_JoystickPrelimited_W.Reset(0.0, 0.0, 2.0);
    m_JoystickLimited_W.Reset(0.0, 0.0, 0.04); // 0.05

    m_EncoderLeft.SetDistancePerPulse(1.0 / 2048.0);
    m_EncoderRight.SetDistancePerPulse(1.0 / 2048.0);

    ActiveBallShifterV1();
    m_State = State::lowGear;
    m_CurrentGearboxRatio = REDUC_V1;*/

    m_JoystickPrelimited_V.Reset(0.0, 0.0, 2.0); // reset des rate limiters
    m_JoystickLimited_V.Reset(0.0, 0.0, 0.06);   // 0.04

    m_JoystickPrelimited_W.Reset(0.0, 0.0, 2.0);
    m_JoystickLimited_W.Reset(0.0, 0.0, 0.06); // 0.0'

    m_EncoderLeft.SetDistancePerPulse(1.0 / 2048.0);
    m_EncoderRight.SetDistancePerPulse(1.0 / 2048.0);

    ActiveBallShifterV1();
    m_State = State::lowGear;
    m_CurrentGearboxRatio = REDUC_V1;

    Reset();
}

void Drivetrain::Set(double v_motor) // set des moteurs
{
    m_MotorLeft1.Set(v_motor);
    m_MotorRight1.Set(v_motor);
}

void Drivetrain::ActiveBallShifterV1() // active ball shifter V1
{
    m_BallShifterSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Drivetrain::ActiveBallShifterV2() // active ball shifter V2
{
    m_BallShifterSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Drivetrain::ChangeBallShifter() // change ball shifter
{
    if (m_BallShifterSolenoid.Get() == frc::DoubleSolenoid::Value::kForward)
    {
        ActiveBallShifterV2();
    }
    else
    {
        ActiveBallShifterV1();
    }
}

double Drivetrain::Calcul_De_Notre_Brave_JM(double forward, double turn, bool wheelSide) // calcule la vitesse des roues
{
    double m_forward = forward;
    double m_turn = turn;

    double left_wheel = m_forward + m_turn * m_sigma;
    double right_wheel = m_forward - m_turn * m_sigma;

    double k;
    k = 1.0 / (NMAX(1, NMAX(NABS(left_wheel), NABS(right_wheel))));
    left_wheel *= k;
    right_wheel *= k;

    if (wheelSide == 0)
        return right_wheel;
    else
        return left_wheel;
}

bool Drivetrain::isUpshiftingAllowed() // mode up, détermine si on peut passer en V2
{
    if ((m_GearShiftingTimeLock == 0.0) /*and (m_GearboxLeftOutAdjustedRpm / m_GearboxRightOutAdjustedRpm < (1 + TURNING_TOLERANCE)) and ((1 - TURNING_TOLERANCE) < m_GearboxLeftOutAdjustedRpm / m_GearboxRightOutAdjustedRpm)*/)
    {
        if (std::abs(m_GearboxesOutAdjustedRpm.m_current) > UP_SHIFTING_POINT_GEARBOXES_OUT_RPM and
            std::abs(m_GearboxesOutAveragedAccelerationRpm2.get()) > UP_SHIFTING_POINT_GEARBOXES_OUT_RPM2 and
            std::abs(m_JoystickRaw_V.m_current) > UP_SHIFTING_POINT_JOYSTICK_V and
            std::abs(m_JoystickRaw_V.m_delta) >= UP_SHIFTING_POINT_JOYSTICK_V_VARIATION)
            return true;
        else
            return false;
    }
}

bool Drivetrain::isKickdownShiftingAllowed() // mode kickdown, détermine si on peut passer en V1
{
    if (std::abs(m_GearboxesOutAdjustedRpm.m_current) < KICKDOWN_SHIFTING_POINT_GEARBOXES_OUT_RPM)
        return true;
    else
        return false;
}

void Drivetrain::ShiftGearUp() // passage de la vitesse en V2
{
    m_JoystickLimited_V.Update(m_JoystickPrelimited_V.m_current);
    ActiveBallShifterV2();
}

void Drivetrain::ShiftGearDown() // passage de la vitesse en V1
{
    m_JoystickLimited_V.Update(m_JoystickPrelimited_V.m_current);
    ActiveBallShifterV1();
}

void Drivetrain::Drive(double joystick_V, double joystick_W, bool brakeButton) //
{
    if (utils::epsilonEquals(joystick_V, 0.0, 0.05))
    {
        joystick_V = 0.0;
    }
    if (utils::epsilonEquals(joystick_W, 0.0, 0.05))
    {
        joystick_W = 0.0;
    }

    m_GearboxRightOutRawRpt.set(m_EncoderRight.GetDistance());
    m_GearboxRightOutAveragedRpt.add(m_GearboxRightOutRawRpt.m_delta);
    m_GearboxLeftOutRawRpt.set(m_EncoderLeft.GetDistance());
    m_GearboxLeftOutAveragedRpt.add(m_GearboxLeftOutRawRpt.m_delta);

    // Vitesses des boites en RPM construitent en combinant les valeurs encodeurs moteurs et through bore
    // TRUST_GEARBOX_OUT_ENCODER représente le coeff de confiance qu'on a dans les encodeurs de sortie de boite et (1-TRUST_GEARBOX_OUT_ENCODER) représente la confiance des encodeurs moteurs
    // m_SuperMotorLeftRpm et m_SuperMotorRightRpm sont déjà exprimé en RPM et m_GearboxRightOutRpt et m_GearboxLeftOutRpt sont en tours/tick (RPT),
    // Il faut donc les convertir en RPM ( * (60/TICK_DT) ).
    // Les m_SuperMotorLeftRpm et m_SuperMotorRightRpm sont les valeurs avant réduction, il faut appliquer le facteur de réduction de boite enclenché
    // pour obtenir une valeur RPM "sortie de boite" (m_CurrentGearboxReductionFactor)
    m_GearboxRightOutAdjustedRpm = (m_GearboxRightOutAveragedRpt.get() * (60 / TICK_DT));
    m_GearboxLeftOutAdjustedRpm = (m_GearboxLeftOutAveragedRpt.get() * (60 / TICK_DT));

    m_GearboxesOutAdjustedRpm.set((m_GearboxRightOutAdjustedRpm + m_GearboxLeftOutAdjustedRpm) / 2.0);
    m_GearboxesOutAveragedAccelerationRpm2.add(m_GearboxesOutAdjustedRpm.m_delta);

    m_JoystickRaw_V.set(joystick_V);
    m_JoystickLimited_V.Update(m_JoystickPrelimited_V.Update(joystick_V));

    m_JoystickRaw_W.set(joystick_W);
    m_JoystickLimited_W.Update(m_JoystickPrelimited_W.Update(joystick_W));

    // décrémentation du temps de verrouillage de la vitesse
    if (m_GearShiftingTimeLock >= TICK_DT)
        m_GearShiftingTimeLock -= TICK_DT;
    else
        m_GearShiftingTimeLock = 0.0;

    switch (m_State)
    {
    case State::lowGear:
    {
        m_sigma = NLERP(0.9, 0.6, NABS(joystick_V)); // 0401
        if (isUpshiftingAllowed() and brakeButton == false)
        {
            m_CurrentGearboxRatio = REDUC_V2;
            ShiftGearUp();
            m_GearShiftingTimeLock = GEARSHIFTING_TIMELOCK;
            m_State = State::highGear;
        }
    }
    break;

    case State::highGear:
    {
        m_sigma = NLERP(0.7, 0.5, NABS(joystick_V)); // 0401

        if (isKickdownShiftingAllowed() or brakeButton == true)
        {
            m_CurrentGearboxRatio = REDUC_V1;
            ShiftGearDown();
            m_GearShiftingTimeLock = GEARSHIFTING_TIMELOCK;
            m_State = State::lowGear;
        }
    }
    break;
    default:
        break;
    }
    if (brakeButton)
    {
        ActiveBallShifterV1();
    }

    m_MotorLeft1.Set(Calcul_De_Notre_Brave_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), 0));
    m_MotorRight1.Set(Calcul_De_Notre_Brave_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), 1));
}

void Drivetrain::DriveAuto(double speed, double rotation, double error)
{
    m_MotorLeft1.Set(rotation + speed);
    m_MotorRight1.Set(-rotation + speed);
    std::cout << -NSIGN(error) * rotation + speed << std::endl;
}

void Drivetrain::Reset()
{
    m_JoystickPrelimited_V.Reset(0.0, 0.0, 2.0); // reset des rate limiters
    m_JoystickLimited_V.Reset(0.0, 0.0, 0.035);  // 0.04

    m_JoystickPrelimited_W.Reset(0.0, 0.0, 2.0);
    m_JoystickLimited_W.Reset(0.0, 0.0, 0.04); // 0.05

    m_EncoderLeft.SetDistancePerPulse(1.0 / 2048.0);
    m_EncoderRight.SetDistancePerPulse(1.0 / 2048.0);

    ActiveBallShifterV1();
    m_State = State::lowGear;
    m_CurrentGearboxRatio = REDUC_V1;

    std::cout << "Reset" << std::endl;
}
