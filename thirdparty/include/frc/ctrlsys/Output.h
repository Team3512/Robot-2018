// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <mutex>

#include "frc/Controller.h"
#include "frc/Notifier.h"
#include "frc/PIDOutput.h"
#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * INode adapter for PIDOutput subclasses.
 *
 * Wraps a PIDOutput object in the INode interface by calling PIDWrite() on it
 * at a regular interval specified in the constructor. This is called in a
 * separate thread.
 */
class Output : public Controller {
public:
    Output(INode& input, PIDOutput& output,
           double period = INode::kDefaultPeriod);
    virtual ~Output() = default;

    void Enable() override;
    void Disable() override;

    void SetRange(double minU, double maxU);

protected:
    virtual void OutputFunc(void);

    friend class OutputGroup;
    friend class RefInput;

private:
    INode& m_input;
    PIDOutput& m_output;
    double m_period;

    Notifier m_thread;
    std::mutex m_mutex;

    double m_minU = -1.0;
    double m_maxU = 1.0;
};

}  // namespace frc
