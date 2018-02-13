// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kLeftRotate,
    kLeftForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoLeftPos() {
    static State state = State::kInit;
    static std::string gameData;

    switch (state) {
        case State::kInit:
            gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (gameData[0] == 'L') {
                robotDrive.SetPositionGoal(
                    168 - kRobotLength / 2);  // Back bumper to middle of robot
            } else {
                robotDrive.SetPositionGoal(228 - kRobotLength / 2);
            }
            robotDrive.SetAngleGoal(0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    robotDrive.ResetGyro();
                    robotDrive.SetAngleGoal(90);
                    state = State::kLeftForward;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(137);  // Estimate
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(90);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(20);  // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(1)) {
                intake.Open();

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();
                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
