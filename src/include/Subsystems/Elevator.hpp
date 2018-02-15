// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/FuncNode.h>
#include <CtrlSys/PIDController.h>
#include <CtrlSys/RefInput.h>
#include <DigitalInput.h>
#include <Notifier.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DriveTrain.hpp"
#include "ES/Service.hpp"
#include "Subsystems/CANTalonGroup.hpp"

enum class ElevatorMode { kPosition, kVelocity };

class Elevator : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    // Sets the voltage of the motors
    void SetVelocity(double velocity);

    // Set encoder distance to 0
    void ResetEncoder();

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Gets encoder values
    double GetHeight();

    // Sets encoder PID setpoints
    void SetHeightReference(double height);

    // Returns encoder PID loop references
    double GetHeightReference() const;

    // Returns whether or not elevator has reached reference
    bool HeightAtReference() const;

    // Gets whether the Hall Effect sensor has triggered
    bool GetHallEffect();

    void HandleEvent(Event event) override;

private:
    WPI_TalonSRX m_elevatorMasterMotor{kElevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{kElevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    Notifier m_notifier;
    // Reference
    frc::RefInput m_heightRef{0.0};

    // Sensors
    frc::DigitalInput m_elevatorHallEffect{kElevatorHallPort};
    frc::FuncNode m_elevatorEncoder{
        [this] { return m_elevatorGearbox.GetPosition(); }};

    frc::PIDController m_elevatorController{
        kElevatorP,        kElevatorI,        kElevatorD,
        m_elevatorEncoder, m_elevatorGearbox, kElevatorControllerPeriod};
};
