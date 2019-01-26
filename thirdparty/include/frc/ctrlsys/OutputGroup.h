// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <vector>

#include "frc/Notifier.h"
#include "frc/ctrlsys/Output.h"

namespace frc {

/**
 * Allows grouping Output instances together to run in one thread.
 *
 * Each output's OutputFunc() is called at a regular interval. This can be used
 * to avoid unnecessary context switches for Output instances that are running
 * at the same sample rate and priority.
 */
class OutputGroup {
public:
    template <class... Outputs>
    explicit OutputGroup(Output& output, Outputs&&... outputs);

    explicit OutputGroup(Output& output);

    virtual ~OutputGroup() = default;

    void Enable(double period = INode::kDefaultPeriod);
    void Disable(void);

protected:
    virtual void OutputFunc(void);

private:
    std::vector<std::reference_wrapper<Output>> m_outputs;
    Notifier m_thread;
};

}  // namespace frc

#include "frc/ctrlsys/OutputGroup.inc"
