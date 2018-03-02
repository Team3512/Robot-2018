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

            robotDrive.SetPositionGoal(300.0 - kRobotLength / 2.0);
            robotDrive.SetAngleGoal(0.0);
            robotDrive.StartClosedLoop();

            elevator.SetHeightReference(kScaleHeight);
            elevator.StartClosedLoop();

            state = State::kInitialForward;
            break;

        case State::kInitialForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(2.0)) {
                robotDrive.SetAngleGoal(90.0);
                state = State::kLeftRotate;
            }
            break;
        case State::kLeftRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                if (platePosition[kScale] == 'L') {
                    robotDrive.SetPositionGoal(20.0);
                } else {
                    robotDrive.SetPositionGoal(137.0);  // Estimate
                }
                state = State::kLeftForward;
            }
            break;
        case State::kLeftForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(2.0)) {
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(-90.0);

                state = State::kFinalRotate;
            }
            break;
        case State::kFinalRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(1.0)) {
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(50.0);  // ESTIMATE

                state = State::kFinalForward;
            }
            break;
        case State::kFinalForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(2.0)) {
                intake.Open();
                robotDrive.ResetGyro();
                robotDrive.SetAngleGoal(180.0);

                state = State::kDoubleRotate;
            }
            break;
        case State::kDoubleRotate:
            if (robotDrive.AtAngleGoal() && autoTimer.HasPeriodPassed(2.0)) {
                elevator.SetHeightReference(kFloorHeight);
                robotDrive.ResetEncoders();
                robotDrive.SetPositionGoal(60.0);

                state = State::kDoubleForward;
            }
            break;
        case State::kDoubleForward:
            if (robotDrive.AtPositionGoal() && autoTimer.HasPeriodPassed(2.0)) {
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
