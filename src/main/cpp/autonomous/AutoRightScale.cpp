// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoRightScale.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightScale::AutoRightScale() { autoTimer.Start(); }

void AutoRightScale::Reset() { state = State::kInit; }

void AutoRightScale::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'R') {
                Robot::drivetrain.SetPositionGoal(324_in - kRobotLength / 2.0);
                Robot::elevator.SetHeightReference(kScaleHeight);
            } else {
                Robot::drivetrain.SetPositionGoal(236.5_in + 10_in -
                                                  kRobotLength / 2.0);
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
                Robot::drivetrain.SetAngleGoal(-88_deg);
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
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
                Robot::drivetrain.SetPositionGoal(199_in);
                Robot::elevator.SetHeightReference(kScaleHeight);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(90_deg);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kScale] == 'R') {
                    // Robot::drivetrain.SetPositionGoal(24_in + 24_in -
                    //                                   kRobotLength / 2.0);
                    Robot::drivetrain.SetPositionGoal(10_in);
                } else {
                    Robot::drivetrain.SetPositionGoal(56_in + 22_in -
                                                      kRobotLength / 2.0);
                }

                state = State::kFinalForward;
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
                state = State::kIdle;
                // state = State::kPrepReverse;
            }
            break;
        case State::kPrepReverse:
            if (autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetPositionGoal(-24_in - 33_in +
                                                      kRobotLength / 2.0);
                } else {
                    Robot::drivetrain.SetPositionGoal(-56_in - 9_in - 12_in +
                                                      kRobotLength / 2.0);
                }
                autoTimer.Reset();
                state = State::kPrepRotate;
            }
            break;
        case State::kPrepRotate:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.SetAngleGoal(-90_deg);
                Robot::elevator.SetHeightReference(0.0);
                autoTimer.Reset();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            Robot::drivetrain.StopClosedLoop();
            break;
    }
}
