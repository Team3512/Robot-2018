// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <hal/HAL.h>

#include <limits>

#include "frc/PIDInterface.h"
#include "frc/PIDOutput.h"
#include "frc/ctrlsys/GainNode.h"
#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/Output.h"
#include "frc/ctrlsys/PIDNode.h"
#include "frc/ctrlsys/RefInput.h"
#include "frc/ctrlsys/SumNode.h"
#include "frc/smartdashboard/SendableBase.h"

namespace frc {

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given PIDOutput.
 *
 * This feedback controller runs in discrete time, so time deltas are not used
 * in the integral and derivative calculations. Therefore, the sample rate
 * affects the controller's behavior for a given set of PID constants.
 */
class PIDController : public SendableBase, public PIDInterface {
public:
    PIDController(double Kp, double Ki, double Kd, INode& input,
                  PIDOutput& output, double period = INode::kDefaultPeriod);
    PIDController(double Kp, double Ki, double Kd, double Kff, INode& input,
                  PIDOutput& output, double period = INode::kDefaultPeriod);
    virtual ~PIDController() = default;

    PIDController(const PIDController&) = delete;
    PIDController& operator=(const PIDController) = delete;

    void SetPID(double Kp, double Ki, double Kd) override;
    void SetPID(double Kp, double Ki, double Kd, double Kff);
    double GetP() const override;
    double GetI() const override;
    double GetD() const override;
    double GetF() const;

    void SetContinuous(bool continuous = true);
    void SetInputRange(double minimumInput, double maximumInput);
    void SetOutputRange(double minimumOutput, double maximumOutput);
    void SetIZone(double maxErrorMagnitude);

    void SetSetpoint(double setpoint) override;
    double GetSetpoint() const override;

    void SetAbsoluteTolerance(
        double tolerance,
        double deltaTolerance = std::numeric_limits<double>::infinity());
    double GetError(void);
    bool OnTarget() const;

    void Enable(void);
    void Disable(void);
    bool IsEnabled() const;
    void SetEnabled(bool enable);

    void Reset() override;

    void InitSendable(SendableBuilder& builder) override;

private:
    RefInput m_refInput{0.0};
    GainNode m_feedforward{0.0, m_refInput};
    SumNode m_sum;
    PIDNode m_pid;
    Output m_output;
    double m_tolerance = 0.05;
    bool m_enabled = false;
};

}  // namespace frc
