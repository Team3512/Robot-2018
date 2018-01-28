// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

enum EventType { kButtonPressed, kClimberSetup, kAtSetHeight, kClimberClimb };

struct Event {
    EventType type;
    int32_t param;
};
