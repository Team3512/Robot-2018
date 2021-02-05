// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoLeftScale.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftScale::AutoLeftScale() { autoTimer.Start(); }

void AutoLeftScale::Reset() { state = State::kInit; }

void AutoLeftScale::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'L') {
                Robot::drivetrain.SetPositionGoal(324_in - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kScaleHeight);
            } else {
                Robot::drivetrain.SetPositionGoal(252_in - kRobotLength / 2.0);
            }
            Robot::drivetrain.SetAngleGoal(0_deg);
            Robot::drivetrain.StartClosedLoop();

            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
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
                Robot::elevator.SetHeightReference(kScaleHeight);
                Robot::drivetrain.SetPositionGoal(199_in);
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
                    // Robot::drivetrain.SetPositionGoal(24_in + 6_in - 6_in +
                    //                                   24_in + 12_in -
                    //                                   kRobotLength / 2.0);
                    Robot::intake.AutoOuttake();
                    state = State::kIdle;
                } else {
                    Robot::drivetrain.SetPositionGoal(56_in + 3_in + 18_in -
                                                      kRobotLength / 2.0);
                    state = State::kFinalForward;
                }
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
            Robot::drivetrain.StopClosedLoop();
            break;
    }
}
