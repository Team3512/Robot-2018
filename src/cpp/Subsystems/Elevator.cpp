// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include <limits>
#include <cmath>
#include <iostream>

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
    m_elevatorGearbox.EnableHardLimits(&m_elevatorBottomHall, nullptr);
    m_elevatorGearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                                       kClimbHeight);
    m_elevatorGearbox.SetHardLimitPressedState(false);
    m_timer.Start();
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

double Elevator::GetVelocity() const { return m_elevatorGearbox.GetSpeed();}

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { return m_elevatorGearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_output.Enable(); }

void Elevator::StopClosedLoop() { m_output.Disable(); }

void Elevator::SetHeightReference(double height) { m_heightRef.Set(height); }

double Elevator::GetHeightReference() const { return m_heightRef.GetOutput(); }

double Elevator::GetAcceleration(double voltage) const {
	// std::cout << "C1: " << (khighG*Kt)/(kR*kr*km) << " C2: " << (voltage-(khighG*m_simulatedVelocity)/kr*Kv) << std::endl;
	return (khighG*Kt)/(kR*kr*kme) * (voltage-(khighG*m_simulatedVelocity)/kr*Kv); // 1/10/
}

void Elevator::CheckEncoderSafety(double joystick_value) { // TODO: make this return a bool; check for high/low gear; if low gear, change km to robot's mass and change high gearing constant to low gearing ratio
	double voltage = joystick_value * RobotController::GetInputVoltage();
	m_simulatedVelocity += GetAcceleration(voltage) * m_timer.Get();
	double m_actualVelocity = m_elevatorGearbox.GetSpeed() * kInPerSecToMPerSec;
	std::cout << "Actual Velocity: " << m_actualVelocity << " Simulated Velocity: " << m_simulatedVelocity << std::endl;
	/*double m_velocityError = std::abs(m_estimatedVelocity - m_actualVelocity);
	if (m_velocityError > #){
		return false;
	} else {
		return true;
	}*/
	m_timer.Reset();
}

bool Elevator::HeightAtReference() const { return m_errorSum.InTolerance(); }

bool Elevator::GetBottomHallEffect() { return m_elevatorBottomHall.Get(); }

void Elevator::HandleEvent(Event event) {
    enum State {
        kIdle,
        kElevatorClimb,
        kElevatorScale,
        kElevatorSwitch,
    };
    static State state = State::kIdle;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kIdle:
            if (event.type == EventType::kElevatorSetSwitch) {
                nextState = State::kElevatorSwitch;
                makeTransition = true;
            } else if (event.type == EventType::kElevatorSetScale) {
                nextState = State::kElevatorScale;
                makeTransition = true;
            } else if (event.type == EventType::kElevatorSetClimb) {
                nextState = State::kElevatorClimb;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.StartPeriodic(0.05);
            }
            break;
        case State::kElevatorSwitch:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kSwitchHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::kElevatorScale:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kScaleHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::kElevatorClimb:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kClimbHeight);
            } else if (HeightAtReference()) {
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
            if (makeTransition) {
                PostEvent(EventType::kExit);
                state = nextState;
                PostEvent(EventType::kEntry);
            }
    }
}
