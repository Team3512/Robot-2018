/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OffsetPIDController.h"

using namespace frc;

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
OffsetPIDController::OffsetPIDController(double Kp, double Ki, double Kd, PIDSource& source,
                             PIDOutput& output, double period)
    : OffsetPIDController(Kp, Ki, Kd, 0.0, 0.0, 0.0, source, output, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param offset The offset applied as a feedforward
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
OffsetPIDController::OffsetPIDController(double Kp, double Ki, double Kd, double offset, PIDSource& source,
                             PIDOutput& output, double period)
    : OffsetPIDController(Kp, Ki, Kd, offset, 0.0, 0.0, source, output, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param offset The offset applied as a feedforward
 * @param Kv     the velocity feedforward coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
OffsetPIDController::OffsetPIDController(double Kp, double Ki, double Kd, double offset, double Kv,
                             PIDSource& source, PIDOutput& output,
                             double period)
    : OffsetPIDController(Kp, Ki, Kd, offset, Kv, 0.0, source, output, period) {}

/**
 * Allocate a PID object with the given constants for P, I, D.
 *
 * @param Kp     the proportional coefficient
 * @param Ki     the integral coefficient
 * @param Kd     the derivative coefficient
 * @param offset The offset applied as a feedforward
 * @param Kv     the velocity feedforward coefficient
 * @param Ka     the acceleration feedforward coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly
 *               effects calculations of the integral and differental terms.
 *               The default is 50ms.
 */
OffsetPIDController::OffsetPIDController(double Kp, double Ki, double Kd, double offset, double Kv,
                             double Ka, PIDSource& source, PIDOutput& output,
                             double period)
    : PIDController(Kp, Ki, Kd, Kv, Ka, source, output, period) {
  m_offset = offset;
}

/**
 * Set the offset applied as a feedforward.
 *
 * @param offset Offset feedforward.
 */
void OffsetPIDController::SetOffset(double offset) { m_offset = offset; }

/**
 * Get the offset applied as a feedforward.
 *
 * @return offset
 */
double OffsetPIDController::GetOffset() const { return m_offset; }

double OffsetPIDController::CalculateFeedForward() {
  return PIDController::CalculateFeedForward() + m_offset;
}
