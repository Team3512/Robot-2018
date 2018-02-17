// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <iostream>

Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {});
    dsDisplay.AddAutoMethod("Autoline", std::bind(&Robot::AutoAutoLine, this));
    dsDisplay.AddAutoMethod("Left Position",
                            std::bind(&Robot::AutoLeftPos, this));
    dsDisplay.AddAutoMethod("Center Position",
                            std::bind(&Robot::AutoCenterPos, this));
    dsDisplay.AddAutoMethod("Right Position",
                            std::bind(&Robot::AutoRightPos, this));

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);

    camera2.SetResolution(640, 480);
    camera2.SetFPS(30);

    server.SetSource(camera1);
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
    elevatorMode = ElevatorMode::kPosition;
}

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
    if (!intake.GetDeploy()) {
        intake.ToggleDeploy();
    }
}

void Robot::TeleopInit() {
    robotDrive.StopClosedLoop();
    elevator.StartClosedLoop();
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {
    DS_PrintOut();

    for (int i = 1; i < 12; i++) {
        if (appendageStick.GetRawButtonPressed(i)) {
            Event event{EventType::kButtonPressed, i};
            climber.PostEvent(event);
            elevator.PostEvent(event);
            intake.PostEvent(event);
        }
    }
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() {
    gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    dsDisplay.ExecAutonomous();
}

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
        intake.SetMotors(MotorState::kIntake);
    }
    if (appendageStick.GetRawButtonPressed(6)) {
        intake.SetMotors(MotorState::kOuttake);
    }
    if (appendageStick.GetRawButtonReleased(4) ||
        appendageStick.GetRawButtonReleased(6)) {
        intake.SetMotors(MotorState::kIdle);
    }

    // Elevator Controls

    switch (elevatorMode) {
        case ElevatorMode::kPosition:  // TODO: change the Height
                                       // References back to constants when
                                       // we know the correct heights
            if (!elevator.GetForwardHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButton(7)) {
                elevator.SetHeightReference(kFloorHeight);
            }

            if (appendageStick.GetRawButton(8)) {
                elevator.SetHeightReference(/*kSwitchHeight*/ -8.0);
            }
            if (appendageStick.GetRawButton(9)) {
                elevator.SetHeightReference(/*kScaleHeight*/ -20.0);
            }
            if (appendageStick.GetRawButton(10)) {
                elevator.SetHeightReference(/*kClimbHeight*/ -80.0);
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StopClosedLoop();
                elevatorMode = ElevatorMode::kVelocity;
            }
            break;
        case ElevatorMode::kVelocity:
            std::cout << elevator.GetHeight() << std::endl;
            elevator.SetVelocity(appendageStick.GetY());
            if (!elevator.GetForwardHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StartClosedLoop();
                elevatorMode = ElevatorMode::kPosition;
            }
    }

    if (appendageStick.GetRawButtonPressed(11)) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }
}

void Robot::DS_PrintOut() {
    robotDrive.Debug();
    /*if (liveGrapher.HasIntervalPassed()){
            liveGrapher.GraphData(elevator.GetHeight(), "Elevator Height");
            liveGrapher.GraphData(elevator.GetHeightReference(), "Elevator
    Reference"); liveGrapher.ResetInterval();
    }*/
}

START_ROBOT_CLASS(Robot)
