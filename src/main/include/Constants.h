// #########################    ID Joystick    #########################

#define ID_JOYSTICK_RIGHT 0
#define ID_JOYSTICK_LEFT 1
#define ID_JOYSTICK_COPILOTER 2

// #########################    ID Motor    #########################

#define ID_MOTOR_DRIVE_TRAIN_RIGHT 1
#define ID_MOTOR_DRIVE_TRAIN_RIGHT_2 2
#define ID_MOTOR_DRIVE_TRAIN_LEFT 3
#define ID_MOTOR_DRIVE_TRAIN_LEFT_2 4

#define ID_MOTOR_FEEDER 5
#define ID_MOTOR_SHOOTER_RIGHT 6
#define ID_MOTOR_SHOOTER_LEFT 7
#define ID_MOTOR_INTAKE 8
#define ID_MOTOR_PLANETARY 9

// #########################    ID Solenoid    #########################
#define ID_SOLENOID_SHIFTER_A 1
#define ID_SOLENOID_SHIFTER_B 0

// #########################    ID Encoder    #########################

#define ID_ENCODER_DRIVE_TRAIN_RIGHT_A 0
#define ID_ENCODER_DRIVE_TRAIN_RIGHT_B 1
#define ID_ENCODER_DRIVE_TRAIN_LEFT_A 2
#define ID_ENCODER_DRIVE_TRAIN_LEFT_B 3

// #########################    ID Sensor    #########################

#define ID_SENSOR_INFRA_FEEDER 0

// #########################    PID Value    #########################

// Turret

#define P_TURRET 0.02
#define I_TURRET 0.0
#define D_TURRET 0.0

// Elevator
#define P_ELEVATOR 2.25
#define I_ELEVATOR 0.0
#define D_ELEVATOR 0.0
#define ELEVATOR_HIGH_CONE 0.97
#define ELEVATOR_MIDDLE_CONE 0.80
#define ELEVATOR_HIGH_CUBE 0.98
#define ELEVATOR_MIDDLE_CUBE 0.43

// Arm
#define P_ARM 0.8 // 0.4
#define I_ARM 0.0
#define D_ARM 0.0

// #########################    VoltageCompensation    #########################

#define DRIVETRAIN_VOLTAGE_COMPENSATION 10.0

// #########################    SmartCurrentLimit    #########################

#define DRIVETRAIN_CURRENT_LIMIT 40

// #########################    Ramp    #########################

#define DRIVETRAIN_RAMP 0.0

// #########################    SetInvertedMotor    #########################

#define DRIVETRAIN_MOTOR_LEFT_INVERTED false
#define DRIVETRAIN_MOTOR_RIGHT_INVERTED true

// #########################    SetDistancePerPulse    #########################

#define TURRET_DISTANCE_PER_PULSE ((1.0 / 2048.0) * (14.0 / 54.0) * 360.0)     // en degré
#define ELEVATOR_DISTANCE_PER_PULSE ((1.0 / 7045.0) * 0.96)                    // en mètre
#define ARM_DISTANCE_PER_PULSE ((1.0 / 2048.0 * 2.0 * 3.14159265358979323846)) // en radian

// #########################   SPEED  #########################

#define SHOOTER_SPEED 0.5
#define INTAKE_SPEED 0.5
#define STOP_INTAKE_SPEED 0.0
#define CATCH_FEEDER_SPEED 0.5
#define EJECT_FEEDER_SPEED 0.5
#define STOP_FEEDER_SPEED 0.0