// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoCenterSwitch.hpp"

#include <cmath>

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoCenterSwitch::AutoCenterSwitch() { autoTimer.Start(); }

void AutoCenterSwitch::Reset() { state = State::kInit; }

void AutoCenterSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::drivetrain.SetPositionGoal(kRobotWidth / 2.0);
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
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetAngleGoal(
                        rad2deg(std::atan2(58.44 + kExchangeOffset,
                                           140 - kRobotWidth)) -
                        10);
                } else {
                    Robot::drivetrain.SetAngleGoal(
                        rad2deg(std::atan2(-76.44 - kExchangeOffset,
                                           140 - kRobotWidth)) +
                        10);
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
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetPositionGoal(
                        std::sqrt(std::pow(58.44 - kExchangeOffset, 2) +
                                  std::pow(140 - kRobotWidth, 2)) +
                        6);
                } else {
                    Robot::drivetrain.SetPositionGoal(
                        std::sqrt(std::pow(76.44 + kExchangeOffset, 2) +
                                  std::pow(140 - kRobotWidth, 2)) -
                        9);
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 0.1) {
                autoTimer.Reset();
                /*
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.ResetGyro();
                    Robot::drivetrain.SetAngleGoal(rad2deg(std::atan2(
                        -140 + kRobotWidth, 58.44 - kExchangeOffset)));
                } else {
                    Robot::drivetrain.ResetGyro();
                    Robot::drivetrain.SetAngleGoal(-rad2deg(std::atan2(
                        -76.44 - kExchangeOffset, 140 - kRobotWidth)) - 10);
                }*/
                Robot::intake.AutoOuttake();

                // state = State::kFinalRotate;
                state = State::kIdle;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(kRobotWidth / 2 - 3);
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 0.1) {
                Robot::intake.SetMotors(MotorState::kOuttake);
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
