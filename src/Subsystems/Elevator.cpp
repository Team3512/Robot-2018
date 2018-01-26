#include "Elevator.hpp"

Elevator::Elevator() {
	m_elevatorGearbox.Set(0.0);
}

void Elevator::SetVelocity(double velocity) {
	m_elevatorGearbox.Set(velocity);
}

void Elevator::SetHeightReference(double position) {
	m_heightRef.Set(position);
}
void Elevator::ResetEncoder() {
	m_elevatorGearbox.ResetEncoder();
}
