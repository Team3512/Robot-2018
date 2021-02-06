// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoCenterScale.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoCenterScale::AutoCenterScale() { autoTimer.Reset(); }

void AutoCenterScale::Reset() { state = State::kInit; }

void AutoCenterScale::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::drivetrain.SetPositionGoal(67.0 -
                                              kRobotLength / 2.0);  // Estimate
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
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetAngleGoal(90.0);
                } else {
                    Robot::drivetrain.SetAngleGoal(-90.0);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetPositionGoal(132.0 - kExchangeOffset -
                                                      kRobotLength / 2.0);
                } else {
                    Robot::drivetrain.SetPositionGoal(132.0 + kExchangeOffset -
                                                      kRobotLength /
                                                          2.0);  // esTIMATE
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetAngleGoal(-90.0);
                } else {
                    Robot::drivetrain.SetAngleGoal(90.0);
                }

                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(260.0);
                autoTimer.Reset();

                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetAngleGoal(-90.0);
                } else {
                    Robot::drivetrain.SetAngleGoal(90.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(40.0 - kRobotLength / 2.0);
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.Open();
                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
