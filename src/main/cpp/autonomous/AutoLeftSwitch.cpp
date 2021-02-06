// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoLeftSwitch.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftSwitch::AutoLeftSwitch() { autoTimer.Start(); }

void AutoLeftSwitch::Reset() { state = State::kInit; }

void AutoLeftSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'L') {
                Robot::drivetrain.SetPositionGoal(168.0 - kRobotLength / 2.0);
            } else {
                Robot::drivetrain.SetPositionGoal(252.0 - kRobotLength / 2.0);
            }
            Robot::drivetrain.SetAngleGoal(0.0);
            Robot::drivetrain.StartClosedLoop();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(90.0);
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'L') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(190.0 - 50.0);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(90.0 - 50.0 + 10.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'L') {
                    Robot::drivetrain.SetPositionGoal(
                        65.0 - kRobotLength / 2.0 - kRobotWidth / 2.0);
                } else {
                    Robot::drivetrain.SetPositionGoal(
                        36.0 + 24.0 - kRobotLength / 2.0);  // 28.0
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 0.1) {
                Robot::intake.AutoOuttake();
                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
