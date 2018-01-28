// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Timer.h>

#include "Service.hpp"

/**
 * Pass event to a handler if a timer expired
 */
class TimerEventGenerator {
public:
    TimerEventGenerator(Event event, double period, bool oneShot = true);

    void Poll(Service& service);

    void Reset();
    bool IsOneShot() const;
    double GetTimePassedSinceLastEvent() const;

private:
    double m_period;
    Event m_original;
    bool m_oneShot;
    Timer m_timer;
};
