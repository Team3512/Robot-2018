// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Timer.h>

#include "es/Service.hpp"

class AutoAutoLine : public Service {
public:
    AutoAutoLine();

    void Reset();

    void HandleEvent(Event event) override;

private:
    frc::Timer autoTimer;

    enum class State { kInit, kMoveForward, kIdle };

    State state;
};
