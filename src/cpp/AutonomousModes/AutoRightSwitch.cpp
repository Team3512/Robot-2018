// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kRightRotate,
    kRightForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoRightSwitchInit() { state = State::kInit; }

void Robot::AutoRightSwitchPeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kFriendlySwitch] == 'R') {
                robotDrive.SetPositionGoal(168.0 - kRobotLength / 2.0);
            } else {
                robotDrive.SetPositionGoal(252.0 - kRobotLength / 2.0);
            }
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
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();
                if (platePosition[kFriendlySwitch] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kRightRotate;
                }
            }
            break;
        case State::kRightRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(190.0);
                autoTimer.Reset();

                state = State::kRightForward;
            }
            break;
        case State::kRightForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                if (platePosition[kFriendlySwitch] == 'R') {
                    robotDrive.SetPositionGoal(65.0 - kRobotLength / 2.0 -
                                               kRobotWidth / 2.0);  // 55
                } else {
                    robotDrive.SetPositionGoal(42.0 - kRobotLength / 2.0);
                }
                autoTimer.Reset();

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                intake.AutoOuttake();
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
    if (robotDrive.PositionError() > 20) {
        state = State::kIdle;
        robotDrive.StopClosedLoop();
        elevator.StopClosedLoop();
    }
}
