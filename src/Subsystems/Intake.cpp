#include "Intake.hpp"

Intake::Intake() {}

void Intake::ToggleOpen() {
	m_IntakeClaw.Set(!m_IntakeClaw.Get());
}

void Intake::ToggleDeploy(bool on) {
	m_IntakeArm.Set(!m_IntakeArm.Get());
}


// IntakeLeft is Ineverted, IntakeRight is not

void Intake::SetMotors(bool on) {
	if (on == true) {
		m_IntakeLeft.Set(0.5);
		m_IntakeRight.Set(-0.5);
	}
	else if(on == false) {
		m_IntakeLeft.Set(0);
		m_IntakeRight.Set(0);
	}
}
