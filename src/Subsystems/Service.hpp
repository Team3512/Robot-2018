#pragma once

#include "Event.hpp"

class Service {
public:
    Service();

    virtual void HandleEvent(Event event) = 0;
}

