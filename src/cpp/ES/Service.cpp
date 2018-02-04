// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "ES/Service.hpp"

Service::Service() { m_thread = std::thread(&Service::RunFramework, this); }

void Service::PostEvent(Event event) {
    {
        std::lock_guard<std::mutex> lk(m);
        m_eventQueue.push_back(event);
    }
    m_ready.notify_one();
}

void Service::RunFramework() {
    while (1) {
        std::unique_lock<std::mutex> lk(m);
        m_ready.wait(lk, [this] { return m_eventQueue.size() > 0; });

        while (m_eventQueue.size() > 0) {
            Event m_event = m_eventQueue.pop_front();
            m.unlock();
            HandleEvent(m_event);
            m.lock();
        }
    }
}
