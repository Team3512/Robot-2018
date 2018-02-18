// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ADXRS450_Gyro.h>
#include <CtrlSys/FuncNode.h>
#include <CtrlSys/RefInput.h>
#include <Drive/DifferentialDrive.h>
#include <Encoder.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "DiffDriveController.hpp"
#include "Subsystems/CANTalonGroup.hpp"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    DriveTrain();
    virtual ~DriveTrain() = default;

    int32_t GetLeftRaw() const;
    int32_t GetRightRaw() const;

    /* Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    // Set encoder distances to 0
    void ResetEncoders();

    // Directly set wheel speeds [0..1] (see GearBox::SetManual(double))
    void SetLeftManual(double value);
    void SetRightManual(double value);

    // Returns encoder distances
    double GetLeftDisplacement() const;
    double GetRightDisplacement() const;

    // Returns encoder rates
    double GetLeftRate() const;
    double GetRightRate() const;

    // Returns robot's current position
    double GetPosition();

    // Return gyro's angle
    double GetAngle();

    // Return gyro's rate
    double GetAngularRate() const;

    // Starts and stops PID loops
    void StartClosedLoop();
    void StopClosedLoop();

    // Sets encoder PID setpoints
    void SetPositionReference(double position);
    void SetAngleReference(double angle);

    // Returns encoder PID loop references
    double GetPosReference() const;
    double GetAngleReference() const;

    // Returns final goals for PID loops
    double GetPosGoal() const;
    double GetAngleGoal() const;

    // Returns whether or not robot has reached its final goal
    bool PosAtGoal() const;
    bool AngleAtGoal() const;

    // Resets gyro
    void ResetGyro();

    // Calibrates gyro
    void CalibrateGyro();

    // Sends print statements for debugging purposes
    void Debug();

private:
    // Left gearbox used in position PID
    WPI_TalonSRX m_leftFront{kLeftDriveMasterID};
    WPI_TalonSRX m_leftRear{kLeftDriveSlaveID};
    CANTalonGroup m_leftGrbx{m_leftFront, m_leftRear};

    // Right gearbox used in position PID
    WPI_TalonSRX m_rightFront{kRightDriveMasterID};
    WPI_TalonSRX m_rightRear{kRightDriveSlaveID};
    CANTalonGroup m_rightGrbx{m_rightFront, m_rightRear};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    ADXRS450_Gyro m_gyro;

    // Control system references
    frc::TrapezoidProfile m_posRef{0.0};
    frc::TrapezoidProfile m_angleRef{0.0};

    // Sensor adapters
    frc::FuncNode m_leftEncoder{[this] { return m_leftGrbx.GetPosition(); }};
    frc::FuncNode m_rightEncoder{[this] { return m_rightGrbx.GetPosition(); }};
    frc::FuncNode m_angleSensor{[this] { return m_gyro.GetAngle(); }};

    frc::DiffDriveController m_controller{
        m_posRef,      m_angleRef, m_leftEncoder, m_rightEncoder,
        m_angleSensor, true,       m_leftGrbx,    m_rightGrbx};
};
