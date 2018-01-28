// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Elevator.hpp"

#include "Robot.hpp"

Elevator::Elevator() { m_elevatorGearbox.Set(0.0); }

void Elevator::SetVelocity(double velocity) { m_elevatorGearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_elevatorGearbox.ResetEncoder(); }

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

void Elevator::HandleEvent(Event event) {
    if (event.type == EventType::kClimberSetup) {
        SetHeightReference(k_climbHeight);
        StartClosedLoop();
        Event atSetHeight{EventType::kAtSetHeight, 0};
        TimerEventGenerator m_elevatorTimer{atSetHeight, 0.1};
        while (!HeightAtReference()) {
            if (event.type == EventType::kAtSetHeight) {
                m_elevatorTimer.Poll(Robot::elevator);
            }
        }
        StopClosedLoop();
        Robot::climber.HandleEvent(atSetHeight);
    }
    if (event.type == EventType::kClimberClimb) {
        StartClosedLoop();
        SetHeightReference(k_scaleHeight);
        Event atSetHeight{EventType::kAtSetHeight, 0};
        TimerEventGenerator m_elevatorTimer{atSetHeight, 0.1};
        while (HeightAtReference()) {
            if (event.type == EventType::kAtSetHeight) {
                m_elevatorTimer.Poll(Robot::elevator);
            }
        }
        StopClosedLoop();
        Robot::climber.HandleEvent(atSetHeight);
    }
}
