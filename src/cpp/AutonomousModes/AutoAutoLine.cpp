// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

enum class State { kInit, kMoveForward, kIdle };

void Robot::AutoAutoLineInit() {}

// Drives forward until passing white line 120 inches away from start
void Robot::AutoAutoLinePeriodic() {
    static State state = State::kInit;

    switch (state) {
        case State::kInit:
            robotDrive.SetPositionReference(168.0 -
                                            kRobotLength / 2.0);  // Estimate
            robotDrive.SetAngleReference(0.0);
            robotDrive.StartClosedLoop();

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
