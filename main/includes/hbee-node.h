#include <stdint.h>

#define REMOTE_LOGS_ENABLED 0
#define BYPASS_GATEWAY 0

#define NODE_FAILURE_DETECTION_ENABLED 0
#define USE_EWMA_FOR_FAILURE_DETECTION 0

#define FAILURE_TIMEOUT (60 * 1000000) // 60 seconds in microseconds

extern uint8_t ENLayer;
extern uint16_t ENId;