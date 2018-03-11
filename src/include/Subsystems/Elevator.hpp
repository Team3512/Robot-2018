// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <CtrlSys/FuncNode.h>
#include <CtrlSys/Output.h>
#include <CtrlSys/PIDNode.h>
#include <CtrlSys/RefInput.h>
#include <CtrlSys/SumNode.h>
#include <DigitalInput.h>
#include <DoubleSolenoid.h>
#include <Notifier.h>
#include <RobotController.h>
#include <Solenoid.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DriveTrain.hpp"
#include "ES/Service.hpp"
#include "Subsystems/CANTalonGroup.hpp"

class Elevator : public Service {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    enum class State { kPosition, kVelocity, kFailedEncoder };

    Elevator();

    // Sets the voltage of the motors
    void SetVelocity(double velocity);

    double GetVelocity() const;

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

    // Toggles the shifter solenoid between forward and reverse
    void Shift();

    // Sets the pawl solenoid to true
    void EngagePawl();

    // Sets the pawl solenoid to false
    void LockPawl();

    // Starts the elevator model
    void StartSimulation();

    // Set simulated position distance to 0
    void ResetSimulation();

    // Gets simulated acceleration
    double GetAcceleration(double voltage) const;

    // Returns whether or not the encoders match the simulated estimates
    bool CheckEncoderSafety(double joystick_value, State state);

    // Sets forward and reverse encoder limits to infinity
    void DisableSoftLimits();

    void HandleEvent(Event event) override;

private:
    WPI_TalonSRX m_elevatorMasterMotor{kElevatorMasterID};
    WPI_TalonSRX m_elevatorSlaveMotor{kElevatorSlaveID};
    CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor,
                                    m_elevatorSlaveMotor};

    frc::Solenoid m_pawl{kPawlPort};
    frc::DoubleSolenoid m_setupSolenoid{kSetupForwardPort, kSetupReversePort};

    State m_state = State::kVelocity;

    bool m_hasBeenZeroed = false;
    double m_u = 0.0;
    double m_simulatedVelocity = 0.0;
    double m_simulatedPosition = 0.0;

    Notifier m_notifier;
    // Reference
    frc::RefInput m_heightRef{0.0};

    frc::Timer m_simTimer;

    // Sensors
    frc::DigitalInput m_elevatorBottomHall{kElevatorBottomHallPort};
    frc::FuncNode m_elevatorEncoder{
        [this] { return m_elevatorGearbox.GetPosition(); }};

    frc::RefInput m_feedForward{kGravityFeedForward};
    frc::SumNode m_errorSum{m_heightRef, true, m_elevatorEncoder, false};
    frc::PIDNode m_pid{kElevatorP,    kElevatorI, kElevatorD,
                       m_feedForward, m_errorSum, kElevatorControllerPeriod};
    frc::Output m_output{m_pid, m_elevatorGearbox, kElevatorControllerPeriod};
};
