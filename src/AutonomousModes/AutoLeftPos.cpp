// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    Init,
    InitialForward,
    RightRotate,
    RightForward,
    FinalRotate,
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
                                                150.0);  // estimate
                robotDrive.SetAngleReference(0);

                state = State::FinalRotate;
            } else {
                robotDrive.SetPositionReference(k_robotLength +
                                                200);  // estimate
                robotDrive.SetAngleReference(0);

                state = State::RightRotate;
            }
            break;
        case State::RightRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.SetAngleReference(90);
                state = State::RightForward;
            }

        case State::RightForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(200);
                state = State::FinalRotate;
            }
            break;
        case State::FinalRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.SetAngleReference(180);

                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
