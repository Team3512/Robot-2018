// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <iostream>
#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kLeftRotate,
    kLeftForward,
    kSecondLeftRotate,
    kSecondLeftForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

static State state;

void Robot::AutoRightScaleInit() { state = State::kInit; }

void Robot::AutoRightScalePeriodic() {
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            std::cout << "Init" << std::endl;
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (platePosition[kScale] == 'R') {
                robotDrive.SetPositionGoal(328.0 - kRobotLength / 2.0);
                elevator.SetHeightReference(kScaleHeight);

                state = State::kInitialForward;
            } else {
                robotDrive.SetPositionGoal(252.0 - kRobotLength / 2.0);
                elevator.SetHeightReference(kScaleHeight);

                state = State::kLeftForward;
            }

            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.StartClosedLoop();

            autoTimer.Reset();
            break;

        case State::kInitialForward:
            std::cout << "Init Forward" << std::endl;
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleGoal(-90.0);

                    state = State::kFinalRotate;
                } else {
                    robotDrive.SetAngleGoal(90.0);

                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            std::cout << "Left Rotate" << std::endl;
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(250.0 + kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            std::cout << "Left Forward" << std::endl;
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kSecondLeftRotate;
            }
            break;
        case State::kSecondLeftRotate:
            std::cout << "Second Left Rotate" << std::endl;
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(60.0 + kRobotWidth / 2.0);
                autoTimer.Reset();

                state = State::kSecondLeftForward;
            }
            break;
        case State::kSecondLeftForward:
            std::cout << "Second Left Forward" << std::endl;
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(90.0);
                autoTimer.Reset();

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            std::cout << "Final Rotate" << std::endl;
            if (robotDrive.AtAngleGoal() ||
                autoTimer.Get() > robotDrive.AngleProfileTimeTotal() + 0.5) {
                robotDrive.ResetEncoders();
                autoTimer.Reset();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetPositionGoal(24.0 + 6.0 - kRobotLength / 2.0);
                } else {
                    robotDrive.SetPositionGoal(40.0 - kRobotWidth / 2.0 -
                                               kRobotLength / 2.0);
                }

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            std::cout << "Final Forward" << std::endl;
            if (robotDrive.AtPositionGoal() ||
                autoTimer.Get() > robotDrive.PositionProfileTimeTotal() + 0.5) {
                intake.AutoOuttake();
                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            std::cout << "Idle" << std::endl;
            break;
    }
    if (robotDrive.PositionError() > 20) {
        state = State::kIdle;
        robotDrive.StopClosedLoop();
        elevator.StopClosedLoop();
    }
}
