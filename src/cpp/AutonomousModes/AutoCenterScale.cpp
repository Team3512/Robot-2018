// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kSecondRotate,
    kThirdForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

void Robot::AutoCenterScale() {
    static State state = State::kInit;
    static std::string gameData;

    switch (state) {
        case State::kInit:
            gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionReference(50);  // Estimate
            robotDrive.SetAngleReference(0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(90);
                } else {
                    robotDrive.SetAngleReference(-90);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                if (gameData[0] == 'R') {
                    robotDrive.SetPositionReference(150);  // ESTIMATE
                } else {
                    robotDrive.SetPositionReference(170);  // ESTIMATE
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetGyro();
                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(-90);
                } else {
                    robotDrive.SetAngleReference(90);
                }
                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (robotDrive.AngleAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(150);  // Estimate
                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1)) {
                robotDrive.ResetGyro();
                if (gameData[0] == 'R') {
                    robotDrive.SetAngleReference(-90);
                } else {
                    robotDrive.SetAngleReference(90);
                }

                state = State::kFinalForward;
            }
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
