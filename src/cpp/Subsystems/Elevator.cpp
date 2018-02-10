// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include "Robot.hpp"

#include <iostream> // TODO: to be removed after testing, if you're seeing this then I failed

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_elevatorGearbox.Set(0.0);
    m_elevatorGearbox.SetDistancePerPulse(k_elevatorDpP);
}

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

double Elevator::GetHeight() { m_elevatorGearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_elevatorController.Enable(); }

void Elevator::StopClosedLoop() { m_elevatorController.Disable(); }

void Elevator::SetHeightReference(double height) {
    m_elevatorController.SetSetpoint(height);
}

double Elevator::GetHeightReference() const {
    return m_elevatorController.GetSetpoint();
}

bool Elevator::HeightAtReference() const {
    return m_elevatorController.OnTarget();
}

bool Elevator::GetHallEffect() { return m_elevatorHallEffect.Get(); }

void Elevator::HandleEvent(Event event) {
    enum State { Idle, ClimberSetup, ClimberClimb, kSetGroundHeight };
    static State state = State::Idle;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::Idle:
            if (event.type == EventType::kClimberSetup) {
                nextState = State::ClimberSetup;
                makeTransition = true;
            } else if (event.type == EventType::kClimberClimb) {
                nextState = State::ClimberClimb;
                makeTransition = true;
            } else if (event.type == EventType::kButtonPressed && event.param == 7){
            	nextState = State::kSetGroundHeight;
            } else if (event.type == EventType::kExit) {
                m_notifier.StartPeriodic(0.05);
            }
            break;
        case State::ClimberSetup:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(k_climbHeight);
            } else if (HeightAtReference()) {
                nextState = State::Idle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::ClimberClimb:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
                SetHeightReference(k_scaleHeight);
            } else if (HeightAtReference()) {
                nextState = State::Idle;
                makeTransition = true;
            } else if (event.type == EventType::kExit) {
                m_notifier.Stop();
                Robot::climber.PostEvent(EventType::kAtSetHeight);
            }
            break;
        case State::kSetGroundHeight:
        	if (event.type == EventType::kEntry){
        		StopClosedLoop();
        		SetVelocity(-0.25);
        	} else if (m_elevatorHallEffect.Get() == false){
        		SetVelocity(0);
        		ResetEncoder();
        		std::cout << "Encoders Reset" << std::endl;
        		nextState = State::Idle;
        	} else if (event.type == EventType::kExit){
        		StartClosedLoop();
        	}
    }
    if (makeTransition) {
        PostEvent(EventType::kExit);
        state = nextState;
        PostEvent(EventType::kEntry);
    }
}
