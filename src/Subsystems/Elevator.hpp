#pragma once

#include <DigitalInput.h>
#include <CtrlSys/RefInput.h>
#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "CANTalonGroup.hpp"
#include "Constants.hpp"
#include "DriveTrain.hpp"

class Elevator {
public:
    using WPI_TalonSRX = ctre::phoenix::motorcontrol::can::WPI_TalonSRX;

    Elevator();

    void ResetEncoder();

    void SetHeightReference(double position);

	void SetVelocity(double velocity);

private:
	WPI_TalonSRX m_elevatorMasterMotor{k_elevatorMasterID};
	WPI_TalonSRX m_elevatorSlaveMotor{k_elevatorSlaveID};
	CANTalonGroup m_elevatorGearbox{m_elevatorMasterMotor, m_elevatorSlaveMotor};

	// Reference
	frc::RefInput m_heightRef{0.0};

	// Sensors
	frc::DigitalInput m_elevatorHallEffect{-1};
	frc::FuncNode m_elevatorEncoder{[this] { return m_elevatorGearbox.GetPosition(); }};
};
