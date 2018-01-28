// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Climber.hpp"

#include "Robot.hpp"

enum class State { Init, Setup, Waiting, Climb, Idle };

void Climber::HandleEvent(Event event) {
    static State state = State::Idle;

    switch (state) {
        case State::Init:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                m_alignmentArms.Set(true);
                Robot::intake.HandleEvent(EventType::kClimberSetup);
                Robot::elevator.HandleEvent(EventType::kClimberSetup);
                state = State::Setup;
            }
            break;
        case State::Setup:
            if (event.type == EventType::kAtSetHeight) {
                m_setupSolenoid.Set(true);
                state = State::Waiting;
            }
            break;
        case State::Waiting:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                Robot::elevator.HandleEvent(EventType::kClimberClimb);
                state = State::Climb;
            }
            break;
        case State::Climb:
            if (event.type == EventType::kAtSetHeight) {
                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
