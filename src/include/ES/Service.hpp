// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "Constants.hpp"
#include "Event.hpp"
#include "circular_buffer.hpp"

class Service {
public:
	/**
	 * Makes a thread for each service
	 */
    Service();
    virtual ~Service();

    /**
     * Does something with, or handles, the event
     */
    virtual void HandleEvent(Event event) = 0;

    /**
     * Posts the event to an event queue and wakes up the condition variable
     */
    void PostEvent(Event event);

private:
    circular_buffer<Event, k_eventQueueSize> m_eventQueue;
    std::condition_variable m_ready;
    std::mutex m_mutex;
    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};

    /**
     * Blocks the thread until the event queue receives at least one event or
     * until the service deconstructs, then handles each event
     */
    void RunFramework();
};
