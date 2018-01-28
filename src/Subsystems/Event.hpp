// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

enum EventType {kNoEvent, kButtonPressed, kClimberSetup, kAtSetHeight, kClimberClimb };

struct Event {
	Event() = default;
    Event(EventType type, int32_t param = 0);

    EventType type = kNoEvent;
    int32_t param = 0;
};
