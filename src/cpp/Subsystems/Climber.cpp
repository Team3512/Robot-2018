// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State { kInit, kSetup, kWaiting, kClimb, kIdle };

void Climber::Shift() {
    if (m_setupSolenoid.Get() == DoubleSolenoid::kForward) {
        m_setupSolenoid.Set(DoubleSolenoid::kReverse);  // Low gear
    } else {
        m_setupSolenoid.Set(DoubleSolenoid::kForward);  // High gear
    }
}

void Climber::Setup() { m_Setup.Set(!m_Setup.Get()); }

void Climber::HandleEvent(Event event) {
    static State state = State::kInit;

    bool makeTransition = false;
    State nextState;

    switch (state) {
        case State::kInit:
            if (event == Event{kButtonPressed, 1}) {
                nextState = State::kSetup;
                makeTransition = true;
            }
            break;
        case State::kSetup:
            if (event.type == EventType::kEntry) {
                Robot::intake.PostEvent(EventType::kElevatorSetClimb);
                Robot::elevator.PostEvent(EventType::kElevatorSetClimb);
            } else if (event.type == EventType::kAtSetHeight) {
                nextState = State::kWaiting;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_setupSolenoid.Set(DoubleSolenoid::kForward);
            }
            break;
        case State::kWaiting:
            if (event == Event{kButtonPressed, 1}) {
                nextState = State::kClimb;
                makeTransition = true;
            }
            break;
        case State::kClimb:
            if (event.type == EventType::kEntry) {
                Robot::elevator.PostEvent(EventType::kElevatorSetClimb);
            } else if (event.type == EventType::kAtSetHeight) {
                nextState = State::kIdle;
                makeTransition = true;
            }
            break;
        case State::kIdle:
            break;
    }
    if (makeTransition) {
        PostEvent(EventType::kExit);
        state = nextState;
        PostEvent(EventType::kEntry);
    }
}
