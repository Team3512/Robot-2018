// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { Init };

// Drives forward until passing white line 120 inches away from start
void Robot::AutoRightPos() {
    static State state = State::Init;

    switch (state) {
        case State::Init:
            robotDrive.StartClosedLoop();

            robotDrive.ResetEncoders();
            robotDrive.ResetGyro();
    }
}
