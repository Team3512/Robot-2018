#pragma once

#include <Solenoid.h>

#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>

#include "Constants.hpp"

class Intake {
public:
	using WPI_TalonSRX = ctre:: phoenix::motorcontrol::can::WPI_TalonSRX;
	Intake();
	void ToggleOpen();
	void ToggleDeploy();
	void SetMotors();
private:
	frc::Solenoid m_intakeClaw{k_intakeClawPort};
	frc::Solenoid m_intakeArm{k_intakeArmPort};
	frc::WPI_TalonSRX m_intakeLeft{k_intakeLeft};
	frc::WPI_TalonSRX m_intakeRight{k_intakeRight};

};
