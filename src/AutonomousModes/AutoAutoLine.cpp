// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { Init, MoveForward, Idle };

// Drives forward until passing white line 120 inches away from start
void Robot::AutoAutoLine() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
            robotDrive.SetPositionReference(k_robotLength + 120.0);
            robotDrive.SetAngleReference(0);

            state = State::MoveForward;
            break;
        case State::MoveForward:
            if (robotDrive.PosAtReference()) {
                robotDrive.StopClosedLoop();

                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
