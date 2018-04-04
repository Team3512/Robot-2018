// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ADXRS450_Gyro.h>
#include <Drive/DifferentialDrive.h>
#include <Encoder.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"
#include "ES/Service.hpp"
#include "PIDController.h"
#include "PIDOutputBuffer.h"
#include "PIDOutputSummer.h"
#include "PIDSourceWrapper.h"
#include "Subsystems/CANTalonGroup.hpp"
#include "TrapezoidProfile.h"

class CANTalonGroup;

/**
 * Provides an interface for this year's drive train
 */
class DriveTrain : public Service {
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
    void SetPositionGoal(double position);
    void SetAngleGoal(double angle);

    // Returns encoder PID loop references
    double GetPosReference();
    double GetAngleReference();

    // Returns final goals for PID loops
    double GetPositionGoal() const;
    double GetAngleGoal() const;

    // Returns whether or not robot has reached its final goal
    bool AtPositionGoal() const;
    bool AtAngleGoal() const;

    double PositionProfileTimeTotal() const;
    double AngleProfileTimeTotal() const;

    // Resets gyro
    void ResetGyro();

    // Calibrates gyro
    void CalibrateGyro();

    // Sends print statements for debugging purposes
    void Debug();

    void HandleEvent(Event event) override;

private:
    // Left gearbox used in position PID
    WPI_TalonSRX m_leftFront{kLeftDriveMasterID};
    WPI_TalonSRX m_leftRear{kLeftDriveSlaveID};
    CANTalonGroup m_leftGrbx{m_leftFront, m_leftRear};
    Encoder m_leftEncoder{kLeftEncoderA, kLeftEncoderB};

    // Right gearbox used in position PID
    WPI_TalonSRX m_rightFront{kRightDriveMasterID};
    WPI_TalonSRX m_rightRear{kRightDriveSlaveID};
    CANTalonGroup m_rightGrbx{m_rightFront, m_rightRear};
    Encoder m_rightEncoder{kRightEncoderA, kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    ADXRS450_Gyro m_gyro;

    frc::PIDSourceWrapper m_positionCalc{[&] {
        return (m_leftGrbx.GetPosition() + m_rightGrbx.GetPosition()) / 2.0;
    }};
    frc::PIDSourceWrapper m_angleSensor{[this] { return m_gyro.GetAngle(); }};

    frc::PIDOutputBuffer m_posControllerBuffer;
    frc::PIDOutputBuffer m_angleControllerBuffer;

    frc::PIDController m_positionController{kPosP,
                                            kPosI,
                                            kPosD,
                                            kVDrive,
                                            kADrive,
                                            m_positionCalc,
                                            m_posControllerBuffer};
    frc::PIDController m_angleController{
        kAngleP,
        kAngleI,
        kAngleD,
        deg2rad(kVAngle* kWheelbaseWidth / 2.0),
        deg2rad(kAAngle* kWheelbaseWidth / 2.0),
        m_angleSensor,
        m_angleControllerBuffer};

    frc::PIDOutputSummer m_leftSummer{m_leftGrbx, m_posControllerBuffer, false,
                                      m_angleControllerBuffer, false};
    frc::PIDOutputSummer m_rightSummer{m_rightGrbx, m_posControllerBuffer, true,
                                       m_angleControllerBuffer, false};

    // Control system references
    frc::TrapezoidProfile m_posRef{m_positionController, kRobotMaxV,
                                   kRobotTimeToMaxV};
    frc::TrapezoidProfile m_angleRef{m_angleController, kRobotMaxRotateRate,
                                     kRobotTimeToMaxRotateRate};
};
