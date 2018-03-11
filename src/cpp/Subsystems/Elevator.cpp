// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
    m_elevatorGearbox.EnableHardLimits(&m_elevatorBottomHall, nullptr);
    m_elevatorGearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                                       -20.0);
    m_elevatorGearbox.SetHardLimitPressedState(false);
    m_simTimer.Start();
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

double Elevator::GetVelocity() const { return m_elevatorGearbox.GetSpeed(); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { return m_elevatorGearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_output.Enable(); }

void Elevator::StopClosedLoop() { m_output.Disable(); }

void Elevator::SetHeightReference(double height) { m_heightRef.Set(height); }

double Elevator::GetHeightReference() const { return m_heightRef.GetOutput(); }

bool Elevator::HeightAtReference() const { return m_errorSum.InTolerance(); }

bool Elevator::GetBottomHallEffect() { return m_elevatorBottomHall.Get(); }

void Elevator::Shift() {
    if (m_setupSolenoid.Get() == DoubleSolenoid::kForward) {
        m_setupSolenoid.Set(DoubleSolenoid::kReverse);  // Low gear
    } else {
        m_setupSolenoid.Set(DoubleSolenoid::kForward);  // High gear
    }
}

bool Elevator::IsLowGear() const {
	if (m_setupSolenoid.Get() == DoubleSolenoid::kReverse){
		return true;
	} else {
		return false;
	}
}

void Elevator::EngagePawl() { m_pawl.Set(true); }

void Elevator::LockPawl() { m_pawl.Set(false); }

bool Elevator::GetPawl() const { return m_pawl.Get(); }

void Elevator::StartSimulation() { m_hasBeenZeroed = true; }

void Elevator::ResetSimulation() { m_simulatedPosition = 0.0; }

double Elevator::GetAcceleration(double voltage) const {
	if (IsLowGear() && GetPawl()){
		return (khighG * Kt) / (kR * kr * kmr) *
		               ((voltage - (klowG * m_simulatedVelocity) / (kr * Kv)));
	} else if (GetPawl()) {
        return (khighG * Kt) / (kR * kr * kme) *
               ((voltage - (klowG * m_simulatedVelocity) / (kr * Kv)));
    } else {
        return (khighG * Kt) / (kR * kr * kme) *
               ((voltage - (khighG * m_simulatedVelocity) / (kr * Kv)));
    }
}

bool Elevator::CheckEncoderSafety(double joystick_value, State state) {
    if (m_hasBeenZeroed) {
        if (state == State::kPosition) {
            m_u = m_pid.GetOutput() * RobotController::GetInputVoltage();
        } else if (state == State::kVelocity) {
            m_u = joystick_value * RobotController::GetInputVoltage();
        }

        m_simulatedVelocity += GetAcceleration(m_u) * m_simTimer.Get();
        m_simulatedPosition += m_simulatedVelocity * m_simTimer.Get();
        m_simTimer.Reset();

        double actualPosition = m_elevatorGearbox.GetPosition();

        return std::abs(actualPosition - (-1*m_simulatedPosition)) <
               -1.0;  // TODO: measure an appropriate tolerance
    } else {
        return true;
    }
}

void Elevator::DisableSoftLimits() {
    m_elevatorGearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                                       std::numeric_limits<double>::infinity());
}

void Elevator::HandleEvent(Event event) {
    bool makeTransition = false;
    State nextState;
    switch (m_state) {
        case State::kPosition:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
            }
            if (event == Event{kButtonPressed, 7}) {
                SetHeightReference(kFloorHeight);
            }
            if (event == Event{kButtonPressed, 8}) {
                SetHeightReference(kSwitchHeight);
            }
            if (event == Event{kButtonPressed, 9}) {
                SetHeightReference(kSecondBlockHeight);
            }
            if (event == Event{kButtonPressed, 10}) {
                SetHeightReference(kScaleHeight);
            }
            if (event == Event{kButtonPressed, 11}) {
                SetHeightReference(kClimbHeight);
            }
            if (event == Event{kButtonPressed, 12}) {
                nextState = State::kVelocity;
                makeTransition = true;
            }
            if (!CheckEncoderSafety(Robot::appendageStick.GetY(), m_state)) {
                StopClosedLoop();
                DisableSoftLimits();
                nextState = State::kFailedEncoder;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                SetHeightReference(GetHeight());
                StopClosedLoop();
            }
            break;
        case State::kVelocity:
            SetVelocity(Robot::appendageStick.GetY());
            if (!CheckEncoderSafety(Robot::appendageStick.GetY(), m_state)) {
                DisableSoftLimits();
                nextState = State::kFailedEncoder;
                makeTransition = true;
            }
            if (event == Event{kButtonPressed, 12}) {
                SetHeightReference(GetHeight());
                nextState = State::kPosition;
                makeTransition = true;
            }
            break;
        case State::kFailedEncoder:
            SetVelocity(Robot::appendageStick.GetY());
    }
    if (Robot::driveStick2.GetRawButton(7) &&
        event == Event{kButtonPressed, 2}) {
        Shift();
    }
    if (Robot::driveStick2.GetRawButton(10) &&
        event == Event{kButtonPressed, 10}) {
        EngagePawl();
    }
    if (makeTransition) {
        HandleEvent(EventType::kExit);
        m_state = nextState;
        HandleEvent(EventType::kEntry);
    }
}
