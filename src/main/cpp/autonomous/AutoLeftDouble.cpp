// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "autonomous/AutoLeftDouble.hpp"

#include <frc/DriverStation.h>

#include "Robot.hpp"

AutoLeftDouble::AutoLeftDouble() { autoTimer.Start(); }

void AutoLeftDouble::Reset() { state = State::kInit; }

void AutoLeftDouble::HandleEvent(Event event) {
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
                Robot::drivetrain.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kLeftRotate;
            }
            break;
        case State::kLeftRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'L') {
                    Robot::drivetrain.SetPositionGoal(20.0);
                } else {
                    Robot::drivetrain.SetPositionGoal(137.0);  // Estimate
                }

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(50.0);  // esTIMATE
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::drivetrain.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.Open();
                Robot::drivetrain.ResetGyro();
                Robot::drivetrain.SetAngleGoal(180.0);
                autoTimer.Reset();

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (Robot::drivetrain.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::drivetrain.AngleProfileTimeTotal() + 1.0) {
                Robot::elevator.SetHeightReference(kFloorHeight);
                Robot::drivetrain.ResetEncoders();
                Robot::drivetrain.SetPositionGoal(60.0);
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
            if (Robot::elevator.HeightAtReference() ||
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
