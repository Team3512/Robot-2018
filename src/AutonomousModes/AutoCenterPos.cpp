// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    k_Init,
    k_InitialForward,
    k_InitialRotate,
    k_SecondForward,
    k_FinalRotate,
    k_FinalForward,
    k_Idle
};

void Robot::AutoCenterPos() {
    static State state = State::k_Init;

    switch (state) {
        case State::k_Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();

            state = State::k_InitialForward;
            break;

        case State::k_InitialForward:
            robotDrive.SetPositionReference(50);  // Estimate

            state = State::k_InitialRotate;
            break;
        case State::k_InitialRotate:
            if (gameData[0] == 'R') {
                robotDrive.SetAngleReference(90);  // Estimate
            } else {
                robotDrive.SetAngleReference(-90);
            }

            state = State::k_SecondForward;

            break;
        case State::k_SecondForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(100);  // Estimate
                state = State::k_FinalRotate;
            }
            break;
        case State::k_FinalRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(-90);
                } else {
                    robotDrive.SetAngleReference(90);
                }
                state = State::k_FinalForward;
            }
            break;
        case State::k_FinalForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(30);  // Estimate
                state = State::k_Idle;
            }
            break;
        case State::k_Idle:
            break;
    }
}
