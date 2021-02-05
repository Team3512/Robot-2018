// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "Robot.hpp"

Drivetrain::Drivetrain() {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftEncoder.SetDistancePerPulse(kDpP);
    m_rightEncoder.SetDistancePerPulse(kDpP);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);

    m_controller.GetPositionPID().SetPID(kPosP, kPosI, kPosD);
    m_controller.GetAnglePID().SetPID(kAngleP, kAngleI, kAngleD);

    m_controller.SetPositionTolerance(1.5, 0.5);
    m_controller.SetAngleTolerance(1.0, 1.75);
}

int32_t Drivetrain::GetLeftRaw() const { return m_leftEncoder.GetRaw(); }

int32_t Drivetrain::GetRightRaw() const { return m_rightEncoder.GetRaw(); }

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, -turn, isQuickTurn);
}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double Drivetrain::GetLeftDisplacement() const {
    return m_leftEncoder.GetDistance();
}

double Drivetrain::GetRightDisplacement() const {
    return m_rightEncoder.GetDistance();
}

double Drivetrain::GetLeftRate() const { return m_leftEncoder.GetRate(); }

double Drivetrain::GetRightRate() const { return m_rightEncoder.GetRate(); }

double Drivetrain::GetPosition() { return m_controller.GetPosition(); }

double Drivetrain::GetAngle() { return m_controller.GetAngle(); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::StartClosedLoop() {
    m_controller.Enable();
    m_drive.SetSafetyEnabled(false);
}

void Drivetrain::StopClosedLoop() {
    m_controller.Disable();
    m_drive.SetSafetyEnabled(true);
}

void Drivetrain::SetPositionGoal(units::inch_t position) {
    m_posRef.SetGoal(position.to<double>());
}

void Drivetrain::SetAngleGoal(units::degree_t angle) {
    m_angleRef.SetGoal(angle.to<double>());
}

double Drivetrain::GetPosReference() {
    return m_posRef.GetPositionNode().GetOutput();
}

double Drivetrain::GetAngleReference() {
    return m_angleRef.GetPositionNode().GetOutput();
}

double Drivetrain::GetPositionGoal() const { return m_posRef.GetGoal(); }

double Drivetrain::GetAngleGoal() const { return m_angleRef.GetGoal(); }

bool Drivetrain::AtPositionGoal() const { return m_controller.AtPosition(); }

bool Drivetrain::AtAngleGoal() const { return m_controller.AtAngle(); }

double Drivetrain::PositionProfileTimeTotal() const {
    return m_posRef.ProfileTimeTotal();
}

double Drivetrain::AngleProfileTimeTotal() const {
    return m_angleRef.ProfileTimeTotal();
}

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

void Drivetrain::Debug() {
    Robot::logger.Log(LogEvent(
        "Left Pos: " + std::to_string(m_leftEncoder.GetDistance()) +
            " Right Pos: " + std::to_string(m_rightEncoder.GetDistance()),
        LogEvent::VERBOSE_DEBUG));
}

void Drivetrain::HandleEvent(Event event) {}
