// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State { kInit, kSetup, kWaiting, kClimb, kIdle };

void Climber::ToggleSetup() {
    if (m_setupSolenoid.Get() == DoubleSolenoid::kForward) {
        m_setupSolenoid.Set(DoubleSolenoid::kReverse);  // Low gear
    } else {
        m_setupSolenoid.Set(DoubleSolenoid::kForward);  // High gear
    }
}

void Climber::GearShift() {
    m_gearShiftSolenoid.Set(!m_gearShiftSolenoid.Get());
}

void Climber::HandleEvent(Event event) {
    if (Robot::driveStick2.GetRawButton(7) &&
        event == Event{kButtonPressed, 2}) {
        ToggleSetup();
    }
    if (Robot::driveStick2.GetRawButton(10) &&
        event == Event{kButtonPressed, 10}) {
        GearShift();
    }
}
