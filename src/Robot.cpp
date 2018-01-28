// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>

Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});
}

void Robot::DisabledInit() { robotDrive.StopClosedLoop(); }

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
}

void Robot::TeleopInit() { robotDrive.StopClosedLoop(); }

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
	DS_PrintOut();
	for (int i = 1; i < 12; i++){
		if (appendageStick.GetRawButtonPressed(i)){
			Event event{EventType::kButtonPressed, i};
			climber.HandleEvent(event);
			elevator.HandleEvent(event);
			intake.HandleEvent(event);
		}
	}
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() { dsDisplay.ExecAutonomous(); }

void Robot::TeleopPeriodic() {
    // Drive Stick Controls
    if (driveStick1.GetRawButton(1)) {
        robotDrive.Drive(driveStick1.GetY() * 0.5, driveStick2.GetX() * 0.5,
                         driveStick2.GetRawButton(2));
    } else {
        robotDrive.Drive(driveStick1.GetY(), driveStick2.GetX(),
                         driveStick2.GetRawButton(2));
    }

    // Intake Controls
    if (appendageStick.GetRawButtonPressed(3)) {
        intake.ToggleOpen();
    }
    if (appendageStick.GetRawButtonPressed(5)) {
        intake.ToggleDeploy();
    }
    if (appendageStick.GetRawButtonPressed(4)) {
        intake.SetMotors(MotorState::k_intake);
    }
    if (appendageStick.GetRawButtonPressed(6)) {
        intake.SetMotors(MotorState::k_outtake);
    }
    if (appendageStick.GetRawButtonReleased(4) ||
        appendageStick.GetRawButtonReleased(6)) {
        intake.SetMotors(MotorState::k_idle);
    }

    // Elevator Controls
    elevator.SetVelocity(appendageStick.GetY());

    if (appendageStick.GetRawButton(7)) {
        elevator.SetHeightReference(k_groundHeight);
    }
    if (appendageStick.GetRawButton(8)) {
        elevator.SetHeightReference(k_switchHeight);
    }
    if (appendageStick.GetRawButton(9)) {
        elevator.SetHeightReference(k_scaleHeight);
    }
    if (appendageStick.GetRawButton(10)) {
        elevator.SetHeightReference(k_climbHeight);
    }

}

void Robot::DS_PrintOut() { robotDrive.Debug(); }

START_ROBOT_CLASS(Robot)
