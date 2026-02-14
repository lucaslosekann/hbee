#include <sx127x.h>

#define BROADCAST_LORA_SF SX127X_SF_9
#define BROADCAST_LORA_FREQ 868500000
#define CH1_LORA_FREQ 868100000
#define CH2_LORA_FREQ 868300000
#define CH3_LORA_FREQ 868700000
#define CH4_LORA_FREQ 869100000
#define CH5_LORA_FREQ 869300000
#define CH6_LORA_FREQ 869500000
#define CH7_LORA_FREQ 869700000

extern long channel;

extern sx127x lora_device;

void setup_lora_device();
void save_id(int32_t id);
int32_t read_id();