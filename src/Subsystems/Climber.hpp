// Copyright (c) 2017-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Joystick.h>
#include <Solenoid.h>
#include <Timer.h>

#include "../Constants.hpp"
#include "Elevator.hpp"
#include "Intake.hpp"
#include "Service.hpp"

class Climber : public Service {
public:
    void Climb();

    void HandleEvent(Event event) override;

private:
    frc::Solenoid m_pawl{k_pawlPort};
    frc::Solenoid m_leftAlignment{k_leftAlignmentPort};
    frc::Solenoid m_rightAlignment{k_rightAlignmentPort};
    frc::Solenoid m_leftRamp{k_leftRampPort};
    frc::Solenoid m_rightRamp{k_rightRampPort};
    frc::Solenoid m_elevatorShifter{k_elevatorShifterPort};

    Timer timer;
};
