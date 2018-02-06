// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

/**
 * Event names to be polled and handled
 */
enum EventType {
    kNoEvent,
    kExit,
    kEntry,
    kButtonPressed,
    kClimberSetup,
    kAtSetHeight,
    kClimberClimb
};

/**
 * Defaults events without specifications of event type and parameter to
 * kNoEvent and 0, respectively
 */
struct Event {
    Event() = default;
    Event(EventType type, int32_t param = 0);  // NOLINT(runtime/explicit)

    EventType type = kNoEvent;
    int32_t param = 0;
};
