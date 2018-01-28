// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#include "Climber.hpp"

#include "Robot.hpp"

enum class State { Init, Setup, Aligned, PawlEngage, Climb, Idle };

void Climber::HandleEvent(Event event) {
    static State state = State::Idle;

    switch (state) {
        case State::Init:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                m_leftAlignment.Set(true);
                m_rightAlignment.Set(true);
                Event setup{EventType::kClimberSetup, 0};
                Robot::intake.HandleEvent(setup);
                Robot::elevator.HandleEvent(setup);
                state = State::Setup;
            }
            break;
        case State::Setup:
            if (event.type == EventType::kAtSetHeight) {
                state = State::Aligned;
            }
            break;
        case State::Aligned:
            m_leftRamp.Set(true);
            m_rightRamp.Set(true);
            m_elevatorShifter.Set(true);
            state = State::PawlEngage;
            break;
        case State::PawlEngage:
            if (event.type == EventType::kButtonPressed && event.param == 1) {
                m_pawl.Set(true);
                state = State::Climb;
            }
            break;
        case State::Climb: {
            Event climb{EventType::kClimberClimb, 0};
            Robot::elevator.HandleEvent(climb);
            state = State::Idle;
        } break;
        case State::Idle:
            if (event.type == EventType::kAtSetHeight) {
                break;
            }
    }
}
