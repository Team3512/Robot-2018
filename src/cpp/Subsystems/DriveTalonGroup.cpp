// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/DriveTalonGroup.hpp"

#include <memory>

#include <ctre/phoenix/MotorControl/SensorCollection.h>

using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

void DriveTalonGroup::Set(double value) {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, value);
}

double DriveTalonGroup::Get() const { return m_canTalons[0].get().Get(); }

void DriveTalonGroup::SetInverted(bool isInverted) {
    for (auto& canTalon : m_canTalons) {
        canTalon.get().SetInverted(isInverted);
    }
}

bool DriveTalonGroup::GetInverted() const {
    return m_canTalons[0].get().GetInverted();
}

void DriveTalonGroup::Disable() {
    m_canTalons[0].get().Set(ControlMode::PercentOutput, 0.0);
}

void DriveTalonGroup::StopMotor() { Disable(); }

void DriveTalonGroup::PIDWrite(double output) { Set(output); }
