// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <DigitalInput.h>
#include <Notifier.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DriveTrain.hpp"
#include "ES/Service.hpp"
#include "OffsetPIDController.h"
#include "PIDSourceWrapper.h"
#include "Subsystems/CANTalonGroup.hpp"

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
    bool GetBottomHallEffect();

    void HandleEvent(Event event) override;

private:
    WPI_TalonSRX m_masterMotor{kElevatorMasterID};
    WPI_TalonSRX m_slaveMotor{kElevatorSlaveID};
    CANTalonGroup m_gearbox{m_masterMotor, m_slaveMotor};

    Notifier m_notifier;

    frc::DigitalInput m_bottomHall{kElevatorBottomHallPort};
    frc::PIDSourceWrapper m_encoder{[this] { return m_gearbox.GetPosition(); }};
    frc::OffsetPIDController m_controller{kElevatorP,
                                          kElevatorI,
                                          kElevatorD,
                                          kGravityFeedForward,
                                          m_encoder,
                                          m_gearbox,
                                          kElevatorControllerPeriod};
};
