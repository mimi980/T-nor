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

#define ID_MOTOR_FEEDER 7

#define ID_MOTOR_SHOOTER_DOWN 8
#define ID_MOTOR_SHOOTER_HIGH 9

#define ID_MOTOR_PLANETARY 10

#define ID_MOTOR_CLIMBER_RIGHT 11
#define ID_MOTOR_CLIMBER_LEFT 12

// #########################    ID Solenoid    #########################
#define ID_SOLENOID_SHIFTER_A 0
#define ID_SOLENOID_SHIFTER_B 1

// #########################    ID Encoder    #########################

#define ID_ENCODER_DRIVE_TRAIN_RIGHT_A 0
#define ID_ENCODER_DRIVE_TRAIN_RIGHT_B 1
#define ID_ENCODER_DRIVE_TRAIN_LEFT_A 2
#define ID_ENCODER_DRIVE_TRAIN_LEFT_B 3

#define ID_ENCODER_PLANETARY_A 6
#define ID_ENCODER_PLANETARY_B 7

#define ID_ENCODER_CLIMBER_A 8
#define ID_ENCODER_CLIMBER_B 9

// #########################    ID Sensor    #########################

#define ID_SENSOR_INFRA_FEEDER 5

// #########################    PID Value    #########################

// Turret
#define PLANETARY_PID_P 0.05
#define PLANETARY_PID_I 0.0
#define PLANETARY_PID_D 0.04
#define PLANETARY_PID_TOLERANCE 1.0

#define CLIMBER_PID_P 0.05
#define CLIMBER_PID_I 0.0
#define CLIMBER_PID_D 0.04
#define CLIMBER_PID_TOLERANCE 1.0

// #########################    VoltageCompensation    #########################

#define DRIVETRAIN_VOLTAGE_COMPENSATION 10.0
#define INTAKE_VOLTAGE_COMPENSATION 12.0
#define FEEDER_VOLTAGE_COMPENSATION 12.0
#define PLANETARY_VOLTAGE_COMPENSATION 10.0
#define SHOOTER_VOLTAGE_COMPENSATION 10.0
#define CLIMBER_VOLTAGE_COMPENSATION 10.0

// #########################    SmartCurrentLimit    #########################

#define DRIVETRAIN_CURRENT_LIMIT 40
#define INTAKE_CURRENT_LIMIT 40
#define FEEDER_CURRENT_LIMIT 40
#define PLANETARY_CURRENT_LIMIT 40
#define SHOOTER_CURRENT_LIMIT 40
#define CLIMBER_CURRENT_LIMIT 40

// #########################    Ramp    #########################

#define DRIVETRAIN_RAMP 0.0
#define INTAKE_RAMP 0.0
#define FEEDER_RAMP 0.0
#define PLANETARY_RAMP 0.0
#define SHOOTER_RAMP 0.1
#define CLIMBER_RAMP 0.1

// #########################    SetInvertedMotor    #########################

#define DRIVETRAIN_MOTOR_LEFT_INVERTED false
#define DRIVETRAIN_MOTOR_RIGHT_INVERTED true
#define INTAKE_MOTOR_INVERTED false
#define FEEDER_MOTOR_INVERTED false
#define PLANETARY_MOTOR_INVERTED false
#define SHOOTER_MOTOR_LEFT_INVERTED true
#define SHOOTER_MOTOR_RIGHT_INVERTED true
#define CLIMBER_MOTOR_LEFT_INVERTED false
#define CLIMBER_MOTOR_RIGHT_INVERTED true

// #########################    SetDistancePerPulse    #########################

#define PLANETARY_DISTANCE_PER_PULSE ((1.0 / 2048.0) / 4.5) * 360.0

// #########################   SPEED  #########################

#define SHOOTER_SPEED 0.5
#define STOP_SHOOTER_SPEED 0.0
#define AMP_SHOOTER_SPEED 0.12
#define NEAR_SPEED_SHOOT 0.5
#define STAGE_SPEED_SHOOT 0.7

#define INTAKE_SPEED 1.0
#define SPIT_INTAKE_SPEED -INTAKE_SPEED
#define STOP_INTAKE_SPEED 0.0

#define CATCH_FEEDER_SPEED 1.0
#define SPIT_FEEDER_SPEED -0.5
#define STOP_FEEDER_SPEED 0.0

#define AMP_ANGLE 100
#define NEAR_ANGLE 38
#define TAKE_ANGLE 17
#define REST_ANGLE 0.0
#define STAGE_ANGLE 18

#define CLIMB_ANGLE 60.0
// #########################   DRIVETRAIN SETTINGS  #########################

#define DRIVE_WHEEL_TRACK_WIDTH_INCHES 22.441
#define TRACK_SCRUB_FACTOR 1.0469745223

// #########################    CAMERA SETTINGS   ###################################

#define CAMERA_PITCH 1.0
#define CAMERA_HEIGHT 1.0
#define TARGET_HEIGHT 1.0
#define SHOOTER_TABLE_SIZE 7

// ##################################################################################
