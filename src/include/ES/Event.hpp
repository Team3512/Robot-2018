// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

/*
 * Event-Naming Prefix Key:
 * 	no prefix indicates system-level events
 * 	'Event' = user level events
 * 	'Done' = reply messaging events
 * 	'Cmd' = command events to another service
 */
enum EventType {
    kNoEvent,
    kExit,
    kEntry,
    kEventButtonPressed,
    kDoneAtSetHeight,
    kCmdIntakeStow,
    kCmdElevatorSetFloor,
    kCmdElevatorSetSwitch,
    kCmdElevatorSetScale,
    kCmdElevatorSetClimb,
};

struct Event {
    Event() = default;
    Event(EventType type, int32_t param = 0);  // NOLINT(runtime/explicit)

    EventType type = kNoEvent;
    int32_t param = 0;
};
