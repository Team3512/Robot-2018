// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include "Service.hpp"

/**
 * Receives events
 */
class EventAcceptor {
public:
    virtual ~EventAcceptor() = default;

    virtual void HandleEvent(Event event) = 0;
};
