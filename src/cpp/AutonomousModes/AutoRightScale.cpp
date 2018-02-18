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

void Robot::AutoRightScale() {
    static State state = State::kInit;
    static std::string gameData;

    switch (state) {
        case State::kInit:
            gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            if (gameData[1] == 'R') {
                robotDrive.SetPositionReference(
                    260 - kRobotLength /
                              2);  // Back bumper to middle of robot (ESTIMATE)
            } else {
                robotDrive.SetPositionReference(228 - kRobotLength / 2);
            }
            robotDrive.SetAngleReference(0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kScaleHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.SetAngleReference(-90);
                if (gameData[0] == 'R') {
                    state = State::kFinalRotate;
                } else {
                    state = State::kLeftRotate;
                }
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(137);  // Estimate
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(90);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                if (gameData[0] == 'R') {
                    robotDrive.SetPositionReference(20);  // ESTIMATE
                } else {
                    robotDrive.SetPositionReference(50);  // ESTIMATE
                }                                         // Estimate
                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
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
