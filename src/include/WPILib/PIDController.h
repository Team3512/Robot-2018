/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>
#include <memory>
#include <string>

#include <support/deprecated.h>
#include <support/mutex.h>

#include "Base.h"
#include "Controller.h"
#include "Filters/LinearDigitalFilter.h"
#include "Notifier.h"
#include "PIDInterface.h"
#include "PIDOutput.h"
#include "PIDSource.h"
#include "SmartDashboard/SendableBase.h"
#include "Timer.h"

namespace frc {

class PIDOutput;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes care of
 * the integral calculations, as well as writing the given PIDOutput.
 *
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
class PIDController : public SendableBase,
                      public PIDInterface,
                      public PIDOutput {
 public:
  PIDController(double Kp, double Ki, double Kd, PIDSource* source,
                PIDOutput* output, double period = 0.05);
  PIDController(double Kp, double Ki, double Kd, double Kv, PIDSource* source,
                PIDOutput* output, double period = 0.05);
  PIDController(double Kp, double Ki, double Kd, double Kv, double Ka,
                PIDSource* source, PIDOutput* output, double period = 0.05);
  PIDController(double Kp, double Ki, double Kd, PIDSource& source,
                PIDOutput& output, double period = 0.05);
  PIDController(double Kp, double Ki, double Kd, double Kv, PIDSource& source,
                PIDOutput& output, double period = 0.05);
  PIDController(double Kp, double Ki, double Kd, double Kv, double Ka,
                PIDSource& source, PIDOutput& output, double period = 0.05);
  ~PIDController() override;

  PIDController(const PIDController&) = delete;
  PIDController& operator=(const PIDController) = delete;

  virtual double Get() const;
  virtual void SetContinuous(bool continuous = true);
  virtual void SetInputRange(double minimumInput, double maximumInput);
  virtual void SetOutputRange(double minimumOutput, double maximumOutput);
  void SetPID(double Kp, double Ki, double Kd) override;
  virtual void SetPID(double Kp, double Ki, double Kd, double Kv, double Ka);
  void SetP(double Kp);
  void SetI(double Ki);
  void SetD(double Kd);
  WPI_DEPRECATED("Use SetV()")
  void SetF(double Kf);
  void SetV(double Kv);
  void SetA(double Ka);
  double GetP() const override;
  double GetI() const override;
  double GetD() const override;
  WPI_DEPRECATED("Use GetV()")
  virtual double GetF() const;
  virtual double GetV() const;
  virtual double GetA() const;

  void SetSetpoint(double setpoint) override;
  double GetSetpoint() const override;
  double GetDeltaSetpoint() const;

  virtual double GetError() const;
  double GetDeltaError() const;

  WPI_DEPRECATED("Use a LinearDigitalFilter as the input and GetError().")
  virtual double GetAvgError() const;

  virtual void SetPIDSourceType(PIDSourceType pidSource);
  virtual PIDSourceType GetPIDSourceType() const;

  WPI_DEPRECATED("Use SetPercentTolerance() instead.")
  virtual void SetTolerance(double percent, double deltaPercent);
  virtual void SetAbsoluteTolerance(
      double tolerance,
      double deltaTolerance = std::numeric_limits<double>::infinity());
  virtual void SetPercentTolerance(
      double tolerance,
      double deltaTolerance = std::numeric_limits<double>::infinity());

  WPI_DEPRECATED("Use a LinearDigitalFilter as the input.")
  virtual void SetToleranceBuffer(int buf = 1);

  virtual bool OnTarget() const;

  void Enable() override;
  void Disable() override;
  void SetEnabled(bool enable);
  bool IsEnabled() const override;

  void Reset() override;

  void PIDWrite(double output) override;

  void InitSendable(SendableBuilder& builder) override;

 protected:
  PIDSource* m_pidInput;
  PIDOutput* m_pidOutput;

  virtual void Calculate();
  virtual double CalculateFeedForward();
  double GetContinuousError(double error) const;

 private:
  // Factor for "proportional" control
  double m_P;

  // Factor for "integral" control
  double m_I;

  // Factor for "derivative" control
  double m_D;

  // Factor for "velocity feedforward" control
  double m_V;

  // Factor for "acceleration feedforward" control
  double m_A;

  // |maximum output|
  double m_maximumOutput = 1.0;

  // |minimum output|
  double m_minimumOutput = -1.0;

  // Maximum input - limit setpoint to this
  double m_maximumInput = 0;

  // Minimum input - limit setpoint to this
  double m_minimumInput = 0;

  // input range - difference between maximum and minimum
  double m_inputRange = 0;

  // Do the endpoints wrap around? eg. Absolute encoder
  bool m_continuous = false;

  // Is the pid controller enabled
  bool m_enabled = false;

  // The prior error (used to compute velocity)
  double m_prevError = 0;

  // The sum of the errors for use in the integral calc
  double m_totalError = 0;

  enum {
    kAbsoluteTolerance,
    kPercentTolerance,
    kNoTolerance
  } m_toleranceType = kNoTolerance;

  // The percetage or absolute error that is considered on target.
  double m_tolerance = 0.05;
  double m_deltaTolerance = std::numeric_limits<double>::infinity();

  double m_setpoint = 0;
  double m_prevSetpoint = 0;
  double m_prevDeltaSetpoint = 0;
  double m_error = 0;
  double m_result = 0;
  double m_period;

  std::shared_ptr<PIDSource> m_origSource;
  LinearDigitalFilter m_filter{nullptr, {}, {}};

  mutable wpi::mutex m_thisMutex;

  // Ensures when Disable() is called, PIDWrite() won't run if Calculate()
  // is already running at that time.
  mutable wpi::mutex m_pidWriteMutex;

  std::unique_ptr<Notifier> m_controlLoop;
  Timer m_setpointTimer;
};

}  // namespace frc
