// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <thread>

#include <wpi/condition_variable.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "Event.hpp"
#include "circular_buffer.hpp"

class Service {
public:
    Service();
    virtual ~Service();

    /**
     * Performs an event for a serviced subsystem
     */
    virtual void HandleEvent(Event event) = 0;

    /**
     * Posts the event to an event queue, wakes up the condition variable, and
     * then notifies the service
     */
    void PostEvent(Event event);

private:
    circular_buffer<Event, kEventQueueSize> m_eventQueue;
    wpi::condition_variable m_ready;
    wpi::mutex m_mutex;
    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};

    /**
     * Blocks the thread until the event queue receives at least one event or
     * until the service deconstructs, then handles each event.
     */
    void RunFramework();
};
