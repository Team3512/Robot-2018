// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <iostream>

#include "Robot.hpp"

enum class State { kInit, kMoveForward, kIdle };

static State state;

void Robot::AutoAutoLineInit() { state = State::kInit; }

// Drives forward until passing white line 120 inches away from start
void Robot::AutoAutoLinePeriodic() {
    switch (state) {
        case State::kInit:
            robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            state = State::kMoveForward;
            std::cout << "Move Forward" << std::endl;
            break;
        case State::kMoveForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.ProfileTimeTotal() + 1.0) {
                robotDrive.StopClosedLoop();
                std::cout << "Idle" << std::endl;
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
