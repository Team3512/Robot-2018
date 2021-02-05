// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoAutoLine.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoAutoLine::AutoAutoLine() { autoTimer.Start(); }

void AutoAutoLine::Reset() { state = State::kInit; }

// Drives forward until passing white line 120 inches away from start
void AutoAutoLine::HandleEvent(Event event) {
    switch (state) {
        case State::kInit:
            Robot::drivetrain.SetPositionGoal(168_in - kRobotLength / 2.0);
            Robot::drivetrain.SetAngleGoal(0_deg);
            Robot::drivetrain.StartClosedLoop();
            autoTimer.Reset();

            state = State::kMoveForward;
            break;
        case State::kMoveForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
