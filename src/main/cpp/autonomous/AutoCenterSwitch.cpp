// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoCenterSwitch.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <units/math.h>

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
            Robot::drivetrain.SetAngleGoal(0_deg);
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
                        units::math::atan2(58.44_in + kExchangeOffset,
                                           140_in - kRobotWidth) -
                        10_deg);
                } else {
                    Robot::drivetrain.SetAngleGoal(
                        units::math::atan2(-76.44_in - kExchangeOffset,
                                           140_in - kRobotWidth) +
                        10_deg);
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
                        units::math::hypot(58.44_in - kExchangeOffset,
                                           140_in - kRobotWidth) +
                        6_in);
                } else {
                    Robot::drivetrain.SetPositionGoal(
                        units::math::hypot(76.44_in + kExchangeOffset,
                                           140_in - kRobotWidth) -
                        9_in);
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
                    Robot::drivetrain.SetAngleGoal(units::math::atan2(
                        -140 + kRobotWidth, 58.44 - kExchangeOffset));
                } else {
                    Robot::drivetrain.ResetGyro();
                    Robot::drivetrain.SetAngleGoal(-units::math::atan2(
                        -76.44 - kExchangeOffset, 140 - kRobotWidth) - 10_deg);
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
                Robot::drivetrain.SetPositionGoal(kRobotWidth / 2 - 3_in);
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
