// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Solenoid.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"

class Intake {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    /**
     * Opens and closes the claw of the Intake
     */
    void ToggleOpen();

    /**
     * Stows and Deploys the Intake back into the frame of the elevator
     */
    void ToggleDeploy();

    void SetMotors(bool on);

private:
    frc::Solenoid m_intakeClaw{k_intakeClawPort};
    frc::Solenoid m_intakeArm{k_intakeArmPort};
    WPI_TalonSRX m_intakeLeft{k_intakeLeftID};
    WPI_TalonSRX m_intakeRight{k_intakeRightID};
};
