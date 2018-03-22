// Copyright (c) 2016-2018 FRC Team 3512. All Rights Reserved.

#include <Timer.h>

#include "ES/Service.hpp"
#include "Robot.hpp"

class AutoCenterScale : public Service {

public:

	void Reset();

	void HandleEvent(Event event) override;
private:
	frc::Timer autoTimer;

enum class State {
    kInit,
    kInitialForward,
    kInitialRotate,
    kSecondForward,
    kSecondRotate,
    kThirdForward,
    kFinalRotate,
    kFinalForward,
    kIdle
};

State state;
};
