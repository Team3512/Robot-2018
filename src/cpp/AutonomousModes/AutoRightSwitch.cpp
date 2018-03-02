// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kRightRotate,
    kRightForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoRightSwitchInit() { state = State::kInit; }

void Robot::AutoRightSwitchPeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'R') {
                robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
            } else {
                robotDrive.SetPositionGoal(236.5 - kRobotLength / 2.0);
            }
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal()) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();

                    state = State::kRightRotate;
                }
            }
            break;
        case State::kRightRotate:
            if (robotDrive.AtAngleGoal()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(236.5 - kRobotWidth / 2.0);
                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (robotDrive.AtPositionGoal()) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(55 - kRobotLength / 2.0);
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal()) {
                intake.SetMotors(MotorState::kOuttake);

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
