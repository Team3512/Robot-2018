// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include <iostream>

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(kElevatorDpP);
    m_elevatorGearbox.EnableHardLimits(&m_elevatorForwardHall,
                                       &m_elevatorReverseHall);
    m_elevatorGearbox.SetLimitPressedState(false);
    m_errorSum.SetTolerance(1.5, 1.5);
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { return m_elevatorGearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_output.Enable(); }

void Elevator::StopClosedLoop() { m_output.Disable(); }

void Elevator::SetHeightReference(double height) { m_heightRef.Set(height); }

double Elevator::GetHeightReference() const { return m_heightRef.GetOutput(); }

bool Elevator::HeightAtReference() const { return m_errorSum.InTolerance(); }

bool Elevator::GetForwardHallEffect() { return m_elevatorForwardHall.Get(); }

void Elevator::HandleEvent(Event event) {
    enum class State {
        kIdle,
        kElevatorFloor,
        kElevatorClimb,
        kElevatorScale,
        kElevatorSwitch,
    };

    static State state = State::kIdle;
    bool makeTransition = false;
    State nextState;

    switch (state) {
        case State::kIdle:
			std::cout << "IN IDLE" << std::endl;
            if (event.type == EventType::kCmdElevatorSetFloor) {
                nextState = State::kElevatorFloor;
                makeTransition = true;
            } else if (event.type == EventType::kCmdElevatorSetSwitch) {
                nextState = State::kElevatorSwitch;
                makeTransition = true;
            } else if (event.type == EventType::kCmdElevatorSetScale) {
                nextState = State::kElevatorScale;
                makeTransition = true;
            } else if (event.type == EventType::kCmdElevatorSetClimb) {
                nextState = State::kElevatorClimb;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.StartPeriodic(0.05);
            }
            break;
        case State::kElevatorFloor:
            if (event.type == EventType::kEntry) {
                SetHeightReference(kFloorHeight);
                StartClosedLoop();
                std::cout << "NOT TRIGGERED" << std::endl;
            } else if (!GetForwardHallEffect()) {
                m_notifier.Stop();
            	std::cout << "TRIGGERED" << std::endl;
            	nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                Robot::climber.PostEvent(EventType::kDoneAtSetHeight);
                std::cout << "EVENT SENT BACK" << std::endl;
            }
            break;
        case State::kElevatorSwitch:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kSwitchHeight);
            } else if (HeightAtReference()) {
                m_notifier.Stop();
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                Robot::climber.PostEvent(EventType::kDoneAtSetHeight);
            }
            break;
        case State::kElevatorScale:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kScaleHeight);
            } else if (HeightAtReference()) {
                m_notifier.Stop();
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                Robot::climber.PostEvent(EventType::kDoneAtSetHeight);
            }
            break;
        case State::kElevatorClimb:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(kClimbHeight);
            } else if (HeightAtReference()) {
                m_notifier.Stop();
                nextState = State::kIdle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                Robot::climber.PostEvent(EventType::kDoneAtSetHeight);
            }
            break;
    }
    if (makeTransition) {
        HandleEvent(EventType::kExit);
        state = nextState;
        HandleEvent(EventType::kEntry);
    }
}
