// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/Elevator.hpp"

#include <limits>

#include "Robot.hpp"

Elevator::Elevator() : m_notifier([&] { Robot::elevator.PostEvent({}); }) {
    m_gearbox.Set(0.0);
    m_gearbox.SetDistancePerPulse(kElevatorDpP);
    m_gearbox.EnableHardLimits(&m_bottomHall, nullptr);
    m_gearbox.EnableSoftLimits(std::numeric_limits<double>::infinity(),
                               kClimbHeight);
    m_gearbox.SetHardLimitPressedState(false);
}

void Elevator::SetVelocity(double velocity) { m_gearbox.Set(velocity); }

void Elevator::ResetEncoder() { m_gearbox.ResetEncoder(); }

double Elevator::GetHeight() { return m_gearbox.GetPosition(); }

void Elevator::StartClosedLoop() { m_controller.Enable(); }

void Elevator::StopClosedLoop() { m_controller.Disable(); }

void Elevator::SetHeightReference(double height) {
    m_controller.SetSetpoint(height);
}

double Elevator::GetHeightReference() const {
    return m_controller.GetSetpoint();
}

bool Elevator::HeightAtReference() const { return m_controller.OnTarget(); }

bool Elevator::GetBottomHallEffect() { return m_bottomHall.Get(); }

void Elevator::HandleEvent(Event event) {
    enum State { kPosition, kVelocity };
    static State state = State::kVelocity;
    bool makeTransition = false;
    State nextState;
    switch (state) {
        case State::kPosition:
            if (event.type == EventType::kEntry) {
                StartClosedLoop();
            }
            if (event == Event{kButtonPressed, 7}) {
                SetHeightReference(kFloorHeight);
            }
            if (event == Event{kButtonPressed, 8}) {
                SetHeightReference(kSwitchHeight);
            }
            if (event == Event{kButtonPressed, 9}) {
                SetHeightReference(kSecondBlockHeight);
            }
            if (event == Event{kButtonPressed, 10}) {
                SetHeightReference(kScaleHeight);
            }
            if (event == Event{kButtonPressed, 11}) {
                SetHeightReference(kClimbHeight);
            }
            if (event == Event{kButtonPressed, 12}) {
                nextState = State::kVelocity;
                makeTransition = true;
            }
            if (event.type == EventType::kExit) {
                SetHeightReference(GetHeight());
                StopClosedLoop();
            }
            break;
        case State::kVelocity:
            SetVelocity(Robot::appendageStick.GetY());
            if (event == Event{kButtonPressed, 12}) {
                SetHeightReference(GetHeight());
                nextState = State::kPosition;
                makeTransition = true;
            }
            break;
    }
    if (makeTransition) {
        HandleEvent(EventType::kExit);
        state = nextState;
        HandleEvent(EventType::kEntry);
    }
}
