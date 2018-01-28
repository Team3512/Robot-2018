#include <stdint.h>

enum EventType {
	kButtonPressed,
	kClimberSetUp
};

struct Event {
	EventType type;
    int32_t param;
};

