#include "esp_timer.h"
#include <stdbool.h>
#include <stdint.h>
void send_data(float voltage, uint8_t *data, uint16_t data_length);
uint64_t get_channel_by_id(uint16_t id);
void setup_message_listening();
void init_send_data_mutex();
void start_tx_queue_consumer();
void start_parent_confirmation_queue_consumer();
typedef enum { ALIVE, SUSPECTED, LOCALLY_DEAD, DEAD } ParentStatus;

typedef struct {
    uint32_t id;
    uint8_t layer;
    float lastVoltage;
    int64_t lastHeard; // Timestamp of the last message received from this parent
    int64_t silence;
    ParentStatus status;
    float ewmaInterval;   // μ_i (intervalo esperado)
    bool ewmaInitialized; // evita usar valor lixo no início
    esp_timer_handle_t confirmTimer;
} parentNode;

#define EWMA_ALPHA 0.2
#define EWMA_K 3
#define EWMA_CONFIRMATION_CONSTANT 4