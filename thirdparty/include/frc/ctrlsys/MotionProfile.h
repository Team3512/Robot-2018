// Copyright (c) 2017-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>
#include <wpi/mutex.h>
#include <tuple>

#include "frc/Timer.h"
#include "frc/ctrlsys/FuncNode.h"
#include "frc/ctrlsys/INode.h"

namespace frc {

/**
 * Base class for all types of motion profile controllers.
 */
class MotionProfile {
public:
    MotionProfile();
    virtual ~MotionProfile() = default;

    INode& GetPositionNode();
    INode& GetVelocityNode();
    INode& GetAccelerationNode();

    virtual void SetGoal(double goal, double currentSource) = 0;
    double GetGoal() const;
    bool AtGoal() const;
    double ProfileTimeTotal() const;

    void Reset(void);

protected:
    using State = std::tuple<double, double, double>;

    virtual State UpdateSetpoint(double currentTime) = 0;

    // Use this to make UpdateSetpoint() and SetGoal() thread-safe
    mutable wpi::mutex m_mutex;

    Timer m_timer;

    double m_goal = 0.0;

    double m_sign;

    // Current reference (displacement, velocity, acceleration)
    State m_ref = std::make_tuple(0.0, 0.0, 0.0);

    frc::FuncNode m_positionNode{[this] {
        std::scoped_lock lock(m_mutex);
        m_ref = UpdateSetpoint(m_timer.Get());
        return std::get<0>(m_ref);
    }};

    frc::FuncNode m_velocityNode{[this] {
        std::scoped_lock lock(m_mutex);
        return std::get<1>(m_ref) * m_sign;
    }};

    frc::FuncNode m_accelerationNode{[this] {
        std::scoped_lock lock(m_mutex);
        return std::get<2>(m_ref) * m_sign;
    }};

    double m_lastTime = 0.0;
    double m_timeTotal = std::numeric_limits<double>::infinity();
};

}  // namespace frc
