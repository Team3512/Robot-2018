#include <stdint.h>

enum EventType {
	kButtonPressed,
	kClimberSetup,
	kAtSetHeight,
	kClimberClimb
};

struct Event {
	EventType type;
    int32_t param;
};

