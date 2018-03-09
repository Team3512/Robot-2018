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

void Robot::AutoCenterScaleInit() {}

void Robot::AutoCenterScalePeriodic() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionReference(67.0 -
                                            kRobotLength / 2.0);  // Estimate
            robotDrive.SetAngleReference(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kSwitchHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleReference(90.0);
                } else {
                    robotDrive.SetAngleReference(-90.0);
                }
                state = State::kInitialRotate;
            }
            break;
        case State::kInitialRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetPositionReference(132.0 + kRobotWidth -
                                                    kRobotLength / 2.0);
                } else {
                    robotDrive.SetPositionReference(
                        132.0 + kExchangeOffset + kRobotWidth -
                        kRobotLength / 2.0);  // ESTIMATE
                }
                state = State::kSecondForward;
            }
            break;
        case State::kSecondForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetGyro();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleReference(-90.0);
                } else {
                    robotDrive.SetAngleReference(90.0);
                }
                state = State::kSecondRotate;
            }
            break;
        case State::kSecondRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(257.0 - kRobotLength / 2.0);
                state = State::kThirdForward;
            }
            break;
        case State::kThirdForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetGyro();
                if (platePosition[kScale] == 'R') {
                    robotDrive.SetAngleReference(-90.0);
                } else {
                    robotDrive.SetAngleReference(90.0);
                }

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(48.0 - kRobotLength / 2.0);

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(1.0)) {
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
