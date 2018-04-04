/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "PIDController.h"

namespace frc {

/**
 * Class implements a PID control loop with a constant feedforward offset.
 *
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
class OffsetPIDController : public PIDController {
 public:
  OffsetPIDController(double Kp, double Ki, double Kd, PIDSource& source,
                PIDOutput& output, double period = 0.05);
  OffsetPIDController(double Kp, double Ki, double Kd, double offset, PIDSource& source,
                PIDOutput& output, double period = 0.05);
  OffsetPIDController(double Kp, double Ki, double Kd, double offset, double Kv, PIDSource& source,
                PIDOutput& output, double period = 0.05);
  OffsetPIDController(double Kp, double Ki, double Kd, double offset, double Kv, double Ka,
                PIDSource& source, PIDOutput& output, double period = 0.05);
  virtual ~OffsetPIDController() = default;

  OffsetPIDController(const OffsetPIDController&) = delete;
  OffsetPIDController& operator=(const OffsetPIDController) = delete;

  void SetOffset(double offset);
  virtual double GetOffset() const;

 protected:
  double CalculateFeedForward() override;

 private:
  // Offset applied as feedforward
  double m_offset;
};

}  // namespace frc
