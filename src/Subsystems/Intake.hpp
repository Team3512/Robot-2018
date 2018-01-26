// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Solenoid.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"

class Intake {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;
    void IntakeOpen();
    void IntakeDeploy();
    void SetMotors(bool on);

private:
    frc::Solenoid m_intakeClaw{k_intakeClawPort};
    frc::Solenoid m_intakeArm{k_intakeArmPort};
    WPI_TalonSRX m_intakeLeft{k_intakeLeft};
    WPI_TalonSRX m_intakeRight{k_intakeRight};
};
