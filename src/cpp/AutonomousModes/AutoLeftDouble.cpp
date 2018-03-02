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
    kDoubleRotate,
    kDoubleForward,
    kSpit,
    kIdle
};

void Robot::AutoLeftDoubleInit() {}

void Robot::AutoLeftDoublePeriodic() {
    static State state = State::kInit;
    static std::string platePosition;

    switch (state) {
        case State::kInit:
            platePosition =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();

            robotDrive.SetPositionReference(300.0 - kRobotLength / 2.0);
            robotDrive.SetAngleReference(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kScaleHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(2.0)) {
                robotDrive.SetAngleReference(90.0);
                state = State::kLeftRotate;
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'L') {
                    robotDrive.SetPositionReference(20.0);
                } else {
                    robotDrive.SetPositionReference(137.0);  // Estimate
                }
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(2.0)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(-90.0);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(50.0);  // ESTIMATE

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(2.0)) {
                intake.Open();
                robotDrive.ResetGyro();
                robotDrive.SetAngleReference(180.0);

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (robotDrive.AngleAtReference() &&
                autoTimer.HasPeriodPassed(2.0)) {
                elevator.SetHeightReference(kFloorHeight);
                robotDrive.ResetEncoders();
                robotDrive.SetPositionReference(60.0);

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (robotDrive.PosAtReference() && autoTimer.HasPeriodPassed(2.0)) {
                intake.Close();
                elevator.SetHeightReference(kSwitchHeight);

                state = State::kSpit;
            }
            break;
        case State::kSpit:
            if (autoTimer.HasPeriodPassed(3.0)) {
                intake.SetMotors(MotorState::kOuttake);

                robotDrive.StopClosedLoop();
                elevator.StopClosedLoop();

                state = State::kIdle;
            }
            break;
        case State::kIdle:
            break;
    }
}
