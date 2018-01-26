// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Intake.hpp"

void Intake::ToggleOpen() { m_intakeClaw.Set(!m_intakeClaw.Get()); }

void Intake::ToggleDeploy() { m_intakeArm.Set(!m_intakeArm.Get()); }

void Intake::SetMotors(bool on) {
    // IntakeLeft is Inverted, IntakeRight is not
    if (on) {
        m_intakeLeft.Set(0.5);
        m_intakeRight.Set(-0.5);
    } else {
        m_intakeLeft.Set(0);
        m_intakeRight.Set(0);
    }
}
