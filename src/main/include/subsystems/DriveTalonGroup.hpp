// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/SpeedController.h>
#include <frc/ctrlsys/INode.h>

/**
 * Represents a gear box with an arbitrary number of motors.
 */
class DriveTalonGroup : public frc::SpeedController {
public:
    using ControlMode = ctre::phoenix::motorcontrol::ControlMode;
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    template <class... CANTalons>
    DriveTalonGroup(WPI_TalonSRX& canTalon, CANTalons&... canTalons);

    // SpeedController interface
    void Set(double value) override;
    double Get() const override;
    void SetInverted(bool isInverted) override;
    bool GetInverted() const override;
    void Disable() override;
    void StopMotor() override;
    void PIDWrite(double output) override;

private:
    std::vector<std::reference_wrapper<WPI_TalonSRX>> m_canTalons;
};

#include "DriveTalonGroup.inc"
