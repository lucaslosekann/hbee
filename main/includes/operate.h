#include <stdint.h>
void send_data(float voltage, uint8_t *data, uint16_t data_length);
uint64_t get_channel_by_id(uint16_t id);
void setup_message_listening();
typedef enum { ALIVE, SUSPECTED, LOCALLY_DEAD, DEAD } ParentStatus;

typedef struct {
    uint32_t id;
    uint8_t layer;
    float lastVoltage;
    int64_t lastHeard; // Timestamp of the last message received from this parent
    ParentStatus status;
} parentNode;