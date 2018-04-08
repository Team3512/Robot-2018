// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoCenterSwitch.hpp"

#include <DriverStation.h>

#include "Robot.hpp"

AutoCenterSwitch::AutoCenterSwitch() { autoTimer.Start(); }

void AutoCenterSwitch::Reset() { state = State::kInit; }

void AutoCenterSwitch::HandleEvent(Event event) {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            Robot::robotDrive.SetPositionGoal(16.5);
            Robot::robotDrive.SetAngleGoal(0.0);
            Robot::robotDrive.StartClosedLoop();

            Robot::elevator.SetHeightReference(kSwitchHeight);
            Robot::elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetAngleGoal(38.290163192243);
                } else {
                    Robot::robotDrive.SetAngleGoal(-45.0);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.SetPositionGoal(
                        96.8297474953);  // Estimate
                } else {
                    Robot::robotDrive.SetPositionGoal(
                        107.48023074);  // Estimate
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    Robot::robotDrive.ResetGyro();
                    Robot::robotDrive.SetAngleGoal(-51.709836807757);
                } else {
                    Robot::robotDrive.ResetGyro();
                    Robot::robotDrive.SetAngleGoal(45.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (Robot::robotDrive.AtAngleGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.AngleProfileTimeTotal() + 1.0) {
                Robot::robotDrive.ResetEncoders();
                Robot::robotDrive.SetPositionGoal(16.5);  // 73.0
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (Robot::robotDrive.AtPositionGoal() ||
                autoTimer.Get() >
                    Robot::robotDrive.PositionProfileTimeTotal() + 1.0) {
                Robot::intake.SetMotors(MotorState::kOuttake);
                Robot::robotDrive.StopClosedLoop();
                Robot::elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
