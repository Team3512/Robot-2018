// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "TimerEventGenerator.hpp"

TimerEventGenerator::TimerEventGenerator(Event event, double period,
                                         bool oneShot) {
    m_period = period;
    m_original = event;
    m_oneShot = oneShot;

    m_timer.Start();
}

void TimerEventGenerator::Poll(Service& service) {
    if (m_timer.HasPeriodPassed(m_period)) {
        /* This is done before calling HandleEvent() so the current state's Exit
         * or the next state's Entry can start it again if desired.
         */
        if (m_oneShot) {
            m_timer.Stop();
            m_timer.Reset();
            m_timer.Start();
        }

        // Force a deep copy to keep the original event name intact
        Event temp = m_original;
        service.HandleEvent(temp);
    }
}

void TimerEventGenerator::Reset() {
    m_timer.Start();
    m_timer.Reset();
}

bool TimerEventGenerator::IsOneShot() const { return m_oneShot; }

double TimerEventGenerator::GetTimePassedSinceLastEvent() const {
    return m_timer.Get();
}
