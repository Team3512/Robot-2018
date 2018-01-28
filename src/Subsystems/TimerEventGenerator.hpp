#pragma once

#include <Timer.h>

#include "Service.hpp"

/**
 * Pass event to a handler if a timer expired
 */
class TimerEventGenerator : public EventGenerator {
public:
    TimerEventGenerator(Event event, double period,
                        bool oneShot = true);

    void Poll(EventAcceptor& acceptor) override;

    void Reset();
    bool IsOneShot() const;
    double GetTimePassedSinceLastEvent() const;

private:
    double m_period;
    Event m_eventName;
    bool m_oneShot;
    Timer m_timer;
};
