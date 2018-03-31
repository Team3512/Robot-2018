// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "AutonomousModes/AutoTimedSwitch.hpp"

#include <string>

#include <DriverStation.h>

#include "Robot.hpp"

AutoTimedSwitch::AutoTimedSwitch() { autoTimer.Start(); }

void AutoTimedSwitch::Reset() { state = State::kForward; }

void AutoTimedSwitch::HandleEvent(Event event) {
    static std::string platePosition;
    platePosition = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    switch (state) {
        case State::kForward:
            Robot::robotDrive.Drive(-0.15, 0, false);
            if (autoTimer.Get() > 4.0) {
                Robot::robotDrive.Drive(0.0, 0, false);
                state = State::kIdle;
                if (platePosition[kFriendlySwitch] = 'L') {
                    Robot::intake.AutoOuttake();
                }
            }
            break;
        case State::kIdle:
            break;
    }
}
