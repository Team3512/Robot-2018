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
                Robot::intake.HandleEvent(EventType::kClimberSetup);
                Robot::elevator.HandleEvent(EventType::kClimberSetup);
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
        case State::Climb:
            Robot::elevator.HandleEvent(EventType::kClimberClimb);
            if (event.type == EventType::kAtSetHeight) {
                state = State::Idle;
            }
            break;
        case State::Idle:
            break;
    }
}
