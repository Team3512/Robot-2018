// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoCenterPos() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.SetPositionGoal(50);  // Estimate
            robotDrive.SetAngleGoal(0);
            elevator.SetHeightReference(kSwitchHeight);
            robotDrive.StartClosedLoop();
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.SetAngleGoal(90);
                } else {
                    robotDrive.SetAngleGoal(-90);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionGoal(90);
                } else {
                    robotDrive.ResetEncoders();
                    robotDrive.SetPositionGoal(110);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(-90);
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(90);
                }
                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(30);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                intake.Open();
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
