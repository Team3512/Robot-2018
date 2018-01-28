#pragma once

#include "Event.hpp"

class Service {
public:
    virtual void HandleEvent(Event event) = 0;
};

