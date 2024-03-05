// #########################    ID Joystick    #########################

#define ID_JOYSTICK_RIGHT 0
#define ID_JOYSTICK_LEFT 1
#define ID_JOYSTICK_COPILOTER 2

// #########################    ID Motor    #########################

#define ID_MOTOR_DRIVE_TRAIN_RIGHT 1
#define ID_MOTOR_DRIVE_TRAIN_RIGHT_2 2
#define ID_MOTOR_DRIVE_TRAIN_LEFT 3
#define ID_MOTOR_DRIVE_TRAIN_LEFT_2 4

#define ID_MOTOR_INTAKE 5
#define ID_MOTOR_FEEDER 6

#define ID_MOTOR_PLANETARY 7

#define ID_MOTOR_SHOOTER_RIGHT 8
#define ID_MOTOR_SHOOTER_LEFT 9

// #########################    ID Solenoid    #########################
#define ID_SOLENOID_SHIFTER_A 0
#define ID_SOLENOID_SHIFTER_B 1

// #########################    ID Encoder    #########################

#define ID_ENCODER_DRIVE_TRAIN_RIGHT_A 0
#define ID_ENCODER_DRIVE_TRAIN_RIGHT_B 1
#define ID_ENCODER_DRIVE_TRAIN_LEFT_A 2
#define ID_ENCODER_DRIVE_TRAIN_LEFT_B 3

#define ID_ENCODER_PLANETARY_A 4
#define ID_ENCODER_PLANETARY_B 5

// #########################    ID Sensor    #########################

#define ID_SENSOR_INFRA_FEEDER 6

// #########################    PID Value    #########################

// Turret
#define PLANETARY_PID_P 0.0
#define PLANETARY_PID_I 0.0
#define PLANETARY_PID_D 0.0

// #########################    VoltageCompensation    #########################

#define DRIVETRAIN_VOLTAGE_COMPENSATION 10.0
#define INTAKE_VOLTAGE_COMPENSATION 10.0
#define FEEDER_VOLTAGE_COMPENSATION 10.0
#define PLANETARY_VOLTAGE_COMPENSATION 10.0
#define SHOOTER_VOLTAGE_COMPENSATION 10.0

// #########################    SmartCurrentLimit    #########################

#define DRIVETRAIN_CURRENT_LIMIT 40
#define INTAKE_CURRENT_LIMIT 40
#define FEEDER_CURRENT_LIMIT 40
#define PLANETARY_CURRENT_LIMIT 40
#define SHOOTER_CURRENT_LIMIT 40

// #########################    Ramp    #########################

#define DRIVETRAIN_RAMP 0.0
#define INTAKE_RAMP 0.0
#define FEEDER_RAMP 0.0
#define PLANETARY_RAMP 0.0
#define SHOOTER_RAMP 0.5

// #########################    SetInvertedMotor    #########################

#define DRIVETRAIN_MOTOR_LEFT_INVERTED false
#define DRIVETRAIN_MOTOR_RIGHT_INVERTED true
#define INTAKE_MOTOR_INVERTED false
#define FEEDER_MOTOR_INVERTED false
#define PLANETARY_MOTOR_INVERTED false
#define SHOOTER_MOTOR_LEFT_INVERTED false
#define SHOOTER_MOTOR_RIGHT_INVERTED true

// #########################    SetDistancePerPulse    #########################

#define PLANETARY_DISTANCE_PER_PULSE 1.0 / 2048.0

// #########################   SPEED  #########################

#define SHOOTER_SPEED 0.5
#define STOP_SHOOTER_SPEED 0.0
#define GOALS_SHOOTER_SPEED 5000

#define INTAKE_SPEED 0.5
#define STOP_INTAKE_SPEED 0.0

#define CATCH_FEEDER_SPEED 0.5
#define EJECT_FEEDER_SPEED 0.5
#define STOP_FEEDER_SPEED 0.0
// #########################   DRIVETRAIN SETTINGS  #########################

#define DRIVE_WHEEL_TRACK_WIDTH_INCHES 22.441
#define TRACK_SCRUB_FACTOR 1.0469745223
