// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);
}

void Robot::DisabledInit() { robotDrive.StopClosedLoop(); }

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
}

void Robot::TeleopInit() { robotDrive.StopClosedLoop(); }

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() { dsDisplay.ExecAutonomous(); }

void Robot::TeleopPeriodic() {
    // Drive Stick Controls
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5);
    } else {
        robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX());
    }
}

START_ROBOT_CLASS(Robot)
