// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    k_Init,
    k_InitialForward,
    k_LeftRotate,
    k_LeftForward,
    k_FinalRotate,
    k_FinalForward,
    k_Idle
};

void Robot::AutoRightPos() {
    static State state = State::k_Init;

    switch (state) {
        case State::k_Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            if (gameData[0] == 'L') {
                    robotDrive.SetPositionReference(150);
            } else {
                    robotDrive.SetPositionReference(200);
                }

            state = State::k_InitialForward;
            break;

        case State::k_InitialForward:
            if (gameData[0] == 'L') {
                    state = State::k_FinalRotate;
            } else {
                    robotDrive.SetAngleReference(-90);
                    state = State::k_LeftForward;
                }
            break;
        case State::k_LeftRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(200);  // Estimate
                state = State::k_LeftForward;
            }
        case State::k_LeftForward:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                robotDrive.SetAngleReference(-90);

                state = State::k_FinalRotate;
            }
            break;
        case State::k_FinalRotate:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(15);  // Estimate
                state = State::k_FinalForward;
            }
            break;
        case State::k_FinalForward:
            if (robotDrive.PosAtReference()) {
                state = State::k_Idle;
            }
            break;
        case State::k_Idle:
            break;
    }
}
