// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Timer.h>

#include "ES/Service.hpp"

class AutoTimedSwitch : public Service {
public:
    AutoTimedSwitch();

    void Reset();

    void HandleEvent(Event event) override;

private:
    frc::Timer autoTimer;

    enum class State {
        kInit,
        kForward,
        kIdle
    };

    State state;
};
