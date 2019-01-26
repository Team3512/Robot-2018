// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>
#include <mutex>

#include "frc/ctrlsys/INode.h"
#include "frc/ctrlsys/NodeBase.h"

namespace frc {

/**
 * Represents an integrator in a control system diagram.
 */
class IntegralNode : public NodeBase {
public:
    IntegralNode(double K, INode& input, double period = kDefaultPeriod);
    virtual ~IntegralNode() = default;

    double GetOutput() override;

    void SetGain(double K);
    double GetGain() const;

    void SetIZone(double maxInputMagnitude);

    void Reset(void);

private:
    double m_gain;
    double m_period;

    double m_total = 0.0;
    double m_maxInputMagnitude = std::numeric_limits<double>::infinity();

    mutable std::mutex m_mutex;
};

}  // namespace frc
