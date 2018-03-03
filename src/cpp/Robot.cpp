// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

std::unique_ptr<Segment[]> Robot::trajectory;
std::unique_ptr<Segment[]> Robot::leftTrajectory;
std::unique_ptr<Segment[]> Robot::rightTrajectory;

Intake Robot::intake;
Elevator Robot::elevator;
Climber Robot::climber;

LiveGrapher Robot::liveGrapher{kLiveGrapherPort};

Robot::Robot() {
    // Auton: does nothing
    dsDisplay.AddAutoMethod("No-op", [] {}, [] {});
    dsDisplay.AddAutoMethod("Autoline Timed",
                            std::bind(&Robot::AutoAutoLineTimedInit, this),
                            std::bind(&Robot::AutoAutoLineTimedPeriodic, this));
    dsDisplay.AddAutoMethod("Autoline",
                            std::bind(&Robot::AutoAutoLineInit, this),
                            std::bind(&Robot::AutoAutoLinePeriodic, this));
    dsDisplay.AddAutoMethod("Left Position Switch",
                            std::bind(&Robot::AutoLeftSwitchInit, this),
                            std::bind(&Robot::AutoLeftSwitchPeriodic, this));
    dsDisplay.AddAutoMethod("Center Position Switch",
                            std::bind(&Robot::AutoCenterSwitchInit, this),
                            std::bind(&Robot::AutoCenterSwitchPeriodic, this));
    dsDisplay.AddAutoMethod("Right Position Switch",
                            std::bind(&Robot::AutoRightSwitchInit, this),
                            std::bind(&Robot::AutoRightSwitchPeriodic, this));
    dsDisplay.AddAutoMethod("Left Position Scale",
                            std::bind(&Robot::AutoLeftScaleInit, this),
                            std::bind(&Robot::AutoLeftScalePeriodic, this));
    dsDisplay.AddAutoMethod("Center Position Scale",
                            std::bind(&Robot::AutoCenterScaleInit, this),
                            std::bind(&Robot::AutoCenterScalePeriodic, this));
    dsDisplay.AddAutoMethod("Right Position Scale",
                            std::bind(&Robot::AutoRightScaleInit, this),
                            std::bind(&Robot::AutoRightScalePeriodic, this));
    server.SetSource(camera1);

    std::array<Waypoint, 3> waypoints;
    waypoints[0] = {-4, -1, d2r(45)};
    waypoints[1] = {-1, 2, 0};
    waypoints[2] = {2, 4, 0};

    //    std::tie(trajectory, leftTrajectory, rightTrajectory) =
    //        GenerateTrajectory(waypoints);

    camera1.SetResolution(640, 480);
    camera1.SetFPS(30);

    // camera2.SetResolution(640, 480);
    // camera2.SetFPS(30);
}

void Robot::DisabledInit() {
    robotDrive.StopClosedLoop();
    robotDrive.ResetGyro();
    robotDrive.ResetEncoders();
    elevator.StopClosedLoop();
    elevator.SetHeightReference(elevator.GetHeight());
    elevatorMode = ElevatorMode::kPosition;
}

void Robot::AutonomousInit() {
    autoTimer.Reset();
    autoTimer.Start();
    robotDrive.ResetEncoders();
    robotDrive.ResetGyro();
    elevator.ResetEncoder();
    intake.Deploy();

    dsDisplay.ExecAutonomousInit();
}

void Robot::TeleopInit() {
	robotDrive.StopClosedLoop();
    elevator.StartClosedLoop();
    intake.Deploy();
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

void Robot::DisabledPeriodic() {
    if (driveStick1.GetRawButtonPressed(12)) {
        robotDrive.ResetEncoders();
    }
}

void Robot::AutonomousPeriodic() { dsDisplay.ExecAutonomousPeriodic(); }

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
    if (appendageStick.GetRawButtonPressed(2)) {
        climber.Shift();
    }

    if (appendageStick.GetRawButtonPressed(3)) {
        if (intake.IsOpen()) {
            intake.Close();
        } else {
            intake.Open();
        }
    }
    if (appendageStick.GetRawButtonPressed(5) &&
        !elevator.GetBottomHallEffect()) {
        if (intake.IsDeployed()) {
            intake.Stow();
        } else {
            intake.Deploy();
        }
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
            if (!elevator.GetBottomHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButton(7)) {
                elevator.SetHeightReference(kFloorHeight);
            }
            if (appendageStick.GetRawButton(9)) {
                elevator.SetHeightReference(kSecondBlockHeight);
            }
            if (appendageStick.GetRawButton(8)) {
                elevator.SetHeightReference(kSwitchHeight);
            }
            if (appendageStick.GetRawButton(10)) {
                elevator.SetHeightReference(kScaleHeight);
            }
            if (appendageStick.GetRawButton(11)) {
                elevator.SetHeightReference(kClimbHeight);
            }
            if (appendageStick.GetRawButtonPressed(5) &&
                !elevator.GetBottomHallEffect()) {
                if (intake.IsDeployed()) {
                    intake.Stow();
                } else {
                    intake.Deploy();
                }
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StopClosedLoop();
                elevatorMode = ElevatorMode::kVelocity;
            }
            break;
        case ElevatorMode::kVelocity:
            elevator.SetVelocity(appendageStick.GetY());
            if (!elevator.GetBottomHallEffect()) {
                elevator.ResetEncoder();
            }
            if (appendageStick.GetRawButtonPressed(12)) {
                elevator.SetHeightReference(elevator.GetHeight());
                elevator.StartClosedLoop();
                elevatorMode = ElevatorMode::kPosition;
            }
    }

    /*if (appendageStick.GetRawButtonPressed(11)) {
        if (server.GetSource() == camera1) {
            server.SetSource(camera2);
        } else {
            server.SetSource(camera1);
        }
    }*/
}

void Robot::DS_PrintOut() {
    if (liveGrapher.HasIntervalPassed()) {
        liveGrapher.GraphData((robotDrive.GetLeftRate() + robotDrive.GetRightRate()) / 2, "Average Velocity");
        liveGrapher.GraphData(robotDrive.GetAngularRate(), "Angle Rate");
        robotDrive.Debug();
        liveGrapher.ResetInterval();
    }
}

START_ROBOT_CLASS(Robot)
