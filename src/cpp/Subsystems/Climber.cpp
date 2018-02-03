// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Climber.hpp"

#include "Robot.hpp"

enum class State { kInit, kSetup, kWaiting, kClimb, kIdle };

void Climber::HandleEvent(Event event) {
    static State state = State::kIdle;

    bool makeTransition = false;
    State nextState;

    switch (state) {
        case State::kInit:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                nextState = State::kSetup;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                Robot::intake.HandleEvent(EventType::kClimberSetup);
                Robot::elevator.HandleEvent(EventType::kClimberSetup);
                m_alignmentArms.Set(true);
            }
            break;
        case State::kSetup:
            if (event.type == EventType::kAtSetHeight) {
                nextState = State::kWaiting;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                m_setupSolenoid.Set(true);
            }
            break;
        case State::kWaiting:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                nextState = State::kClimb;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                Robot::elevator.HandleEvent(EventType::kClimberClimb);
            }
            break;
        case State::kClimb:
            if (event.type == EventType::kAtSetHeight) {
                nextState = State::kIdle;
                makeTransition = true;
            }
            break;
        case State::kIdle:
            break;
    }
    if (makeTransition) {
        HandleEvent(EventType::kExit);
        state = nextState;
        HandleEvent(EventType::kEntry);
    }
}
