// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoLeftPriority.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftPriority::AutoLeftPriority() { autoTimer.Start(); }

void AutoLeftPriority::Reset() { state = State::kInit; }

void AutoLeftPriority::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'L') {
                Robot::drivetrain.SetPositionGoal(324_in - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kScaleHeight);

                state = State::kInitialForward;
            } else if (platePosition[kFriendlySwitch] == 'L' &&
                       platePosition[kScale] == 'R') {
                Robot::drivetrain.SetPositionGoal(168_in - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kAutoSwitch;
            } else {
                Robot::drivetrain.SetPositionGoal(168_in - kRobotLength / 2.0);

                state = State::kAutoLine;
            }

            Robot::drivetrain.SetAngleGoal(0_deg);
            Robot::drivetrain.StartClosedLoop();

            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.SetAngleGoal(90_deg);
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kRightRotate;
                }
            }
            break;
        case State::kRightRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(200_in + kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(-90_deg);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    // Robot::drivetrain.SetPositionGoal(
                    //     24_in + 6_in - 6_in + 24_in - kRobotLength / 2.0);
                    Robot::intake.AutoOuttake();
                    state = State::kIdle;
                } else {
                    Robot::drivetrain.SetPositionGoal(
                        40_in - kRobotWidth / 2.0 - kRobotLength / 2.0);
                    state = State::kFinalForward;
                }
                autoTimer.Reset();
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.AutoOuttake();
                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoLine:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();
                autoTimer.Reset();

                state = State::kIdle;
            }
            break;
        case State::kAutoSwitch:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(90_deg);
                autoTimer.Reset();

                state = State::kAutoSwitchRotate;
            }
            break;
        case State::kAutoSwitchRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(65_in - kRobotLength / 2.0 -
                                                  kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kAutoSwitchForward;
            }
            break;
        case State::kAutoSwitchForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.AutoOuttake();

                autoTimer.Reset();

                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            Robot::drivetrain.StopClosedLoop();
            break;
    }
}
