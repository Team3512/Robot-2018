// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

// Includes definition for Talons and etc that connect to the RoboRIO

/* Order of subsystem constants:
 * > Motor IDs
 * > Solenoid Ports
 * > Limit switches
 * > Distance per pulse
 * > PID
 * > Other (i.e. miscellaneous constants)
 */

// DS port
constexpr int kDsPort = 5800;

// LiveGrapher host port
constexpr int kLiveGrapherPort = 3513;

// MJPEG server port
constexpr int kMjpegServerPort = 1180;

// Event Queue Size
constexpr int kEventQueueSize = 8;

/*
 * Joystick and buttons
 */

// Joystick ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;

/*
 * DriveTrain
 */

// DriveTrain GearBox ID
constexpr int kLeftDriveMasterID = 1;
constexpr int kLeftDriveSlaveID = 2;
constexpr int kRightDriveMasterID = 3;
constexpr int kRightDriveSlaveID = 4;

// Distance per Pulse
constexpr double kLeftDpP = 169.0 / ((10926.0 + 10910.0) / 2.0);
constexpr double kRightDpP = 169.0 / ((10913.0 + 10802.0) / 2.0);

// DriveTrain position PID
constexpr double kPosP = 0.05;
constexpr double kPosI = 0.00;
constexpr double kPosD = 0.024;

// Drive trapezoid profile constants
constexpr double kRobotMaxV = 227.2;               // in/sec
constexpr double kRobotTimeToMaxV = 3.0;           // sec
constexpr double kRobotMaxRotateRate = 50.0;       // deg/sec
constexpr double kRobotTimeToMaxRotateRate = 2.0;  // sec

// Drive motor feedforwards
constexpr double kV = 1.0 / 227.2;  // 1 / max velocity
constexpr double kA = 0.1;          // (V - (kV * v + Vmin)) / a

// DriveTrain angle PID
constexpr double kAngleP = 0.05;
constexpr double kAngleI = 0.00;
constexpr double kAngleD = 0.012;

// Physical Robot Constants
constexpr int kRobotLength = 0;
constexpr double kWheelbaseWidth = 0.6;
constexpr double kDegreesToRadians = 3.1415926535897932 / 180.0;

// CheesyDrive constants
constexpr double kLowGearSensitive = 0.75;
constexpr double kTurnNonLinearity = 1.0;
constexpr double kInertiaDampen = 2.5;
constexpr double kInertiaHighTurn = 3.0;
constexpr double kInertiaLowTurn = 3.0;

/*
 * Intake
 */

// Talon IDs
constexpr int kIntakeLeftID = 5;
constexpr int kIntakeRightID = 6;

// Solenoid Ports
constexpr int kIntakeClawPort = 1;
constexpr int kIntakeArmPort = 2;

/*
 * Elevator
 */

// Elevator GearBox ID
constexpr int kElevatorMasterID = 7;
constexpr int kElevatorSlaveID = 8;

// Hall Effect Sensor Port
constexpr int kElevatorForwardHallPort = 1;
constexpr int kElevatorReverseHallPort = 0;

// Distance per Pulse
constexpr double kElevatorDpP = 0.00142230843;

// Elevator PID
constexpr double kElevatorP = 0.09;
constexpr double kElevatorI = 0.004;
constexpr double kElevatorD = 0.0;
constexpr double kElevatorControllerPeriod = 0.02;
constexpr double kGravityFeedForward = -0.01;

// Elevator Setpoints
constexpr double kFloorHeight = 0.0;
constexpr double kSwitchHeight = -12.0;
constexpr double kScaleHeight = -60.0;
constexpr double kClimbHeight = -80.0;

/*
 * Climber
 */

// Climber Solenoid ports
constexpr int kAlignmentArmsPort = 3;
constexpr int kSetupForwardPort = 4;
constexpr int kSetupReversePort = 5;

