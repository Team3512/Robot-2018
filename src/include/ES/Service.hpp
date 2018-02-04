// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <Notifier.h>

#include "Constants.hpp"
#include "Event.hpp"
#include "circular_buffer.hpp"

class Service {
public:
    Service();

    virtual void HandleEvent(Event event) = 0;

    void PostEvent(Event event);

private:
    void RunFramework();

    circular_buffer<Event, k_eventQueueSize> m_eventQueue;
    std::condition_variable m_ready;
    std::mutex m_mutex;
    std::thread m_thread;
};
