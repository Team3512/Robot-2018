// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kLeftRotate,
    kLeftForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoLeftPos() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.StartClosedLoop();
            elevator.ResetEncoder();
            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            elevator.SetHeightReference(kSwitchHeight);
            if (gameData[0] == 'R') {
                robotDrive.SetPositionReference(
                    168 - kRobotLength / 2);  // Back bumper to middle of robot
            } else {
                robotDrive.SetPositionReference(228 - kRobotLength / 2);
            }

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference()) {
                if (gameData[0] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleReference(90);
                    state = State::kLeftForward;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(150);  // Estimate
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(90);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(20);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference()) {
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
