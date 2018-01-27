// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    Init,
    InitialForward,
    RightRotate,
    RightForward,
    FinalRotate,
    FinalForward,
    Idle
};

// Drives forward until passing white line 120 inches away from start
void Robot::AutoLeftPos() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();

            state = State::InitialForward;
            break;

        case State::InitialForward:
            if (gameData[0] == 'L') {
                robotDrive.SetPositionReference(k_robotLength +
                                                150);  // Estimate
                robotDrive.SetAngleReference(0);

                state = State::FinalRotate;
            } else {
                robotDrive.SetPositionReference(k_robotLength +
                                                200);  // Estimate
                robotDrive.SetAngleReference(0);

                state = State::RightRotate;
            }
            break;
        case State::RightRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.SetAngleReference(90);
                state = State::RightForward;
            }
            break;
        case State::RightForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(200);  // Estimate
                state = State::FinalRotate;
            }
            break;
        case State::FinalRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                robotDrive.SetAngleReference(90);

                state = State::FinalForward;
            }
            break;
        case State::FinalForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(15);  // Estimate
                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
