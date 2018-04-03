// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoCenterSwitchInit() { state = State::kInit; }

void Robot::AutoCenterSwitchPeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionGoal(67.0 - kRobotLength / 2.0);
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            autoTimer.Reset();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetAngleGoal(90.0);
                } else {
                    robotDrive.SetAngleGoal(-90.0);
                }

                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetPositionGoal(66.0 -
                                               kExchangeOffset);  // Estimate
                } else {
                    robotDrive.SetPositionGoal(70.0 +             // 60.0
                                               kExchangeOffset);  // Estimate
                }

                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(-90.0);
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(90.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(79.0 - kRobotLength / 2.0);  // 73.0
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetPositionGoal(82.0 - kRobotLength / 2.0);
                }
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                intake.SetMotors(MotorState::kOuttake);
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
    if (robotDrive.PositionError() < 20) {
        state = State::kIdle;
        robotDrive.StopClosedLoop();
        elevator.StopClosedLoop();
    }
}
