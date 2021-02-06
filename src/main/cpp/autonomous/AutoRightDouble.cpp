// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoRightDouble.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoRightDouble::AutoRightDouble() { autoTimer.Start(); }

void AutoRightDouble::Reset() { state = State::kInit; }

void AutoRightDouble::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::drivetrain.SetPositionGoal(236.5 - kRobotLength / 2.0);
            Robot::drivetrain.SetAngleGoal(0.0);
            Robot::drivetrain.StartClosedLoop();

            Robot::elevator.SetHeightReference(kScaleHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kRightRotate;
            }
            break;
        case State::kRightRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetPositionGoal(20.0 -
                                                      kRobotLength / 2.0);
                } else if (platePosition[kFriendlySwitch] == 'R' &&
                           platePosition[kScale] == 'L') {
                    Robot::drivetrain.SetPositionGoal(20.0 -
                                                      kRobotLength / 2.0);

                } else {
                    Robot::drivetrain.SetPositionGoal(236.5 -
                                                      kRobotLength / 2.0);
                }

                state = State::kFirstForward;
            }
            break;
        case State::kFirstForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R' &&
                    platePosition[kScale] == 'R') {
                    Robot::drivetrain.SetAngleGoal(90);
                } else {
                    Robot::drivetrain.SetAngleGoal(-90);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(40.0 - kRobotLength /
                                                             2.0);  // esTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.SetMotors(MotorState::kOuttake);
                Robot::drivetrain.ResetGyro();
                autoTimer.Reset();

                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetAngleGoal(-180.0);
                } else {
                    Robot::drivetrain.SetAngleGoal(0.0);
                }

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::intake.SetMotors(MotorState::kIdle);
                Robot::intake.Open();
                Robot::elevator.SetHeightReference(kFloorHeight);
                Robot::drivetrain.ResetEncoders();
                if (platePosition[kScale] == 'R' &&
                    platePosition[kFriendlySwitch] == 'R') {
                    Robot::drivetrain.SetPositionGoal(60.0);
                } else {
                    Robot::drivetrain.SetPositionGoal(10.0);
                }
                autoTimer.Reset();

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.Close();
                Robot::elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (Robot::elevator.HeightAtReference() &&
                autoTimer.HasPeriodPassed(3.0)) {
                Robot::intake.SetMotors(MotorState::kOuttake);

                Robot::drivetrain.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
