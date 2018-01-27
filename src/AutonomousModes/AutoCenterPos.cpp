// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State {
    Init,
    InitialForward,
    InitialRotate,
    SecondForward,
    FinalRotate,
    FinalForward,
    Idle
};

// Drives forward until passing white line 120 inches away from start
void Robot::AutoCenterPos() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();

            state = State::InitialForward;
            break;

        case State::InitialForward:
            robotDrive.SetPositionReference(50); // Estimate

                state = State::InitialRotate;
            break;
        case State::InitialRotate:
            if (gameData[0] == 'R') {
                robotDrive.SetAngleReference(90);  // Estimate
            } else {
                robotDrive.SetAngleReference(-90);
            }

            state = State::SecondForward;

            break;
        case State::SecondForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(100);  // Estimate
                state = State::FinalRotate;
            }
            break;
        case State::FinalRotate:
            if (robotDrive.PosAtReference()) {
                robotDrive.ResetEncoders();  // For Simplicity

                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(-90);
                } else {
                    robotDrive.SetAngleReference(90);
                }
                state = State::FinalForward;
            }
            break;
        case State::FinalForward:
            if (robotDrive.AngleAtReference()) {
                robotDrive.SetPositionReference(30);  // Estimate
                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
