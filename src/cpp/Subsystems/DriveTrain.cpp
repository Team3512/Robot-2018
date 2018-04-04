// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include "Subsystems/DriveTrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#include "Robot.hpp"

DriveTrain::DriveTrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftEncoder.SetDistancePerPulse(kDpP);
    m_rightEncoder.SetDistancePerPulse(kDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);

    m_positionController.SetTolerance(1.5, 0.5);
    m_angleController.SetTolerance(1.0, 1.75);
}

int32_t DriveTrain::GetLeftRaw() const { return m_leftEncoder.GetRaw(); }

int32_t DriveTrain::GetRightRaw() const { return m_rightEncoder.GetRaw(); }

void DriveTrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void DriveTrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void DriveTrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void DriveTrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double DriveTrain::GetLeftDisplacement() const {
    return m_leftEncoder.GetDistance();
}

double DriveTrain::GetRightDisplacement() const {
    return m_rightEncoder.GetDistance();
}

double DriveTrain::GetLeftRate() const { return m_leftEncoder.GetRate(); }

double DriveTrain::GetRightRate() const { return m_rightEncoder.GetRate(); }

double DriveTrain::GetPosition() { return m_positionCalc.PIDGet(); }

double DriveTrain::GetAngle() { return m_angleSensor.PIDGet(); }

double DriveTrain::GetAngularRate() const { return m_gyro.GetRate(); }

void DriveTrain::StartClosedLoop() {
    m_positionController.Enable();
    m_angleController.Enable();
    m_posRef.Enable();
    m_angleRef.Enable();
    m_leftSummer.Enable();
    m_rightSummer.Enable();
    m_drive.SetSafetyEnabled(false);
}

void DriveTrain::StopClosedLoop() {
    m_positionController.Disable();
    m_angleController.Disable();
    m_posRef.Disable();
    m_angleRef.Disable();
    m_leftSummer.Disable();
    m_rightSummer.Disable();
    m_drive.SetSafetyEnabled(true);
}

void DriveTrain::SetPositionGoal(double position) {
    m_posRef.SetGoal(position);
}

void DriveTrain::SetAngleGoal(double angle) { m_angleRef.SetGoal(angle); }

double DriveTrain::GetPosReference() { return m_posRef.GetReference(); }

double DriveTrain::GetAngleReference() { return m_angleRef.GetReference(); }

double DriveTrain::GetPositionGoal() const { return m_posRef.GetGoal(); }

double DriveTrain::GetAngleGoal() const { return m_angleRef.GetGoal(); }

bool DriveTrain::AtPositionGoal() const {
    return m_positionController.OnTarget();
}

bool DriveTrain::AtAngleGoal() const { return m_angleController.OnTarget(); }

double DriveTrain::PositionProfileTimeTotal() const {
    return m_posRef.GetTotalTime();
}

double DriveTrain::AngleProfileTimeTotal() const {
    return m_angleRef.GetTotalTime();
}

void DriveTrain::ResetGyro() { m_gyro.Reset(); }

void DriveTrain::CalibrateGyro() { m_gyro.Calibrate(); }

void DriveTrain::Debug() {
    std::cout << "Left Pos: " << m_leftGrbx.GetPosition()
              << " Right Pos: " << m_rightGrbx.GetPosition() << std::endl;
}

void DriveTrain::HandleEvent(Event event) {
    if (Robot::driveStick1.GetRawButton(1)) {
        Drive(Robot::driveStick1.GetY() * 0.5, Robot::driveStick2.GetX() * 0.5,
              Robot::driveStick2.GetRawButton(2));
    } else {
        Drive(Robot::driveStick1.GetY(), Robot::driveStick2.GetX(),
              Robot::driveStick2.GetRawButton(2));
    }
}
