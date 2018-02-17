// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State {
    kInit,
    kElevatorReset,
    kIntakeStow,
    kSetup,
    kWaiting,
    kClimb,
    kIdle
};

void Climber::HandleEvent(Event event) {
    static State state = State::kInit;

    bool makeTransition = false;
    State nextState;

    switch (state) {
        case State::kInit:
            if (event.type == EventType::kEventButtonPressed &&
                event.param == 1) {
                nextState = State::kElevatorReset;
                makeTransition = true;
            }
            break;
        case State::kElevatorReset:
            if (event.type == EventType::kEntry) {
                Robot::elevator.PostEvent(EventType::kCmdElevatorSetFloor);
            } else if (event.type == EventType::kDoneAtSetHeight) {
                nextState = State::kIntakeStow;
                makeTransition = true;
            }
            break;
        case State::kIntakeStow:
            if (event.type == EventType::kEntry) {
                Robot::intake.PostEvent(EventType::kCmdIntakeStow);
                m_timer.Start();
                m_timer.Reset();
            } else if (m_timer.HasPeriodPassed(3.0)) {
                nextState = State::kSetup;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_timer.Stop();
            }
            break;
        case State::kSetup:
            if (event.type == EventType::kEntry) {
                Robot::elevator.PostEvent(EventType::kCmdElevatorSetClimb);
                m_alignmentArms.Set(true);
            } else if (event.type == EventType::kDoneAtSetHeight) {
                nextState = State::kWaiting;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_setupSolenoid.Set(DoubleSolenoid::kForward);
            }
            break;
        case State::kWaiting:
            if (event.type == EventType::kEventButtonPressed &&
                event.param == 1) {
                nextState = State::kClimb;
                makeTransition = true;
            }
            break;
        case State::kClimb:
            if (event.type == EventType::kEntry) {
                Robot::elevator.PostEvent(EventType::kCmdElevatorSetScale);
            } else if (event.type == EventType::kDoneAtSetHeight) {
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
