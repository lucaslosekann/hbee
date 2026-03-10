#include <mosquitto.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h> 


#include "loragw_aux.h"
#include "loragw_hal.h"

#define MQTT_HOST "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC "HBEE2026gateway/rx"

static volatile bool exit_sig = false;
static struct mosquitto *mosq = NULL;

static void sig_handler(int sigio) { exit_sig = true; }

/* Converte payload para string HEX */
void payload_to_hex(uint8_t *payload, uint16_t size, char *output) {
    for (int i = 0; i < size; i++) {
        sprintf(output + (i * 2), "%02X", payload[i]);
    }
}

int send_beacon(void) {
    struct lgw_pkt_tx_s txpkt;
    memset(&txpkt, 0, sizeof(txpkt));

    uint8_t payload[2] = {0x01, 0x00};

    txpkt.freq_hz = 868500000; // mesma freq do RF0
    txpkt.rf_chain = 0;        // usar RF0
    txpkt.rf_power = 2;       // potência em dBm
    txpkt.modulation = MOD_LORA;
    txpkt.bandwidth = BW_125KHZ;
    txpkt.datarate = DR_LORA_SF9;
    txpkt.coderate = CR_LORA_4_5;
    txpkt.invert_pol = false;
    txpkt.preamble = 8;
    txpkt.no_crc = false;
    txpkt.no_header = false;

    txpkt.size = 2;
    txpkt.payload[0] = payload[0];
    txpkt.payload[1] = payload[1];

    txpkt.tx_mode = IMMEDIATE;
    txpkt.count_us = 0;

    if (lgw_send(&txpkt) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to send beacon\n");
        return -1;
    }

    /* Espera terminar transmissão */
    wait_ms(1500);

    printf("Initializing message sent\n");
    return 0;
}
typedef struct {
    int32_t freq_hz;
    int32_t  freq_offset;
    uint32_t count_us;
    uint16_t crc;
} pkt_id_t;

int main(int argc, char *argv[]) {
    if(argc != 2){
        printf("Usage: %s <csv_file_path>\n", argv[0]);
        return -1;
    }
    /* Signal handling */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    /* ---------------- MQTT INIT ---------------- */
    mosquitto_lib_init();
    mosq = mosquitto_new("sx1302_rx", true, NULL);

    if (!mosq) {
        printf("MQTT init failed\n");
        return -1;
    }

    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        printf("MQTT connect failed\n");
        return -1;
    }

    printf("Connected to MQTT broker\n");

    /* ---------------- SX1302 INIT ---------------- */

    /* Board reset */
    if (system("./reset_lgw.sh start") != 0) {
        printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
        exit(EXIT_FAILURE);
    }
        


    struct lgw_conf_board_s boardconf;
    memset(&boardconf, 0, sizeof(boardconf));
    boardconf.lorawan_public = false;
    boardconf.clksrc = 0;
    boardconf.com_type = LGW_COM_SPI;
    strcpy(boardconf.com_path, "/dev/spidev0.0");
    lgw_board_setconf(&boardconf);

    /* RF0 */
    struct lgw_conf_rxrf_s rfconf;
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable = true;
    rfconf.freq_hz = 868500000;
    rfconf.type = LGW_RADIO_TYPE_SX1250;
    rfconf.tx_enable = true;
    lgw_rxrf_setconf(0, &rfconf);

    /* RF1 */
    memset(&rfconf, 0, sizeof(rfconf));
    rfconf.enable = true;
    rfconf.freq_hz = 869500000;   // Centro 869.5
    rfconf.type = LGW_RADIO_TYPE_SX1250;
    rfconf.tx_enable = false;     // Pode deixar false se só vai receber
    lgw_rxrf_setconf(1, &rfconf);

    int32_t if_freq_rf0[4] = {
        -400000,
        -200000,
        0,
        200000
    };

    for (int i = 0; i < 4; i++) {
        struct lgw_conf_rxif_s ifconf;
        memset(&ifconf, 0, sizeof(ifconf));
        ifconf.enable = true;
        ifconf.rf_chain = 0;
        ifconf.freq_hz = if_freq_rf0[i];
        ifconf.datarate = DR_LORA_SF9;
        lgw_rxif_setconf(i, &ifconf);
    }

    int32_t if_freq_rf1[4] = {
        -400000,
        -200000,
        0,
        200000
    };

    for (int i = 0; i < 4; i++) {
        struct lgw_conf_rxif_s ifconf;
        memset(&ifconf, 0, sizeof(ifconf));
        ifconf.enable = true;
        ifconf.rf_chain = 1;
        ifconf.freq_hz = if_freq_rf1[i];
        ifconf.datarate = DR_LORA_SF9;
        lgw_rxif_setconf(i + 4, &ifconf);
    }

    if (lgw_start() != LGW_HAL_SUCCESS) {
        printf("Failed to start concentrator\n");
        return -1;
    }

    FILE *csv_file = fopen(argv[1], "a");
    if (!csv_file) {
        perror("Failed to open CSV file");
        return -1;
    }
    printf("Sending startup beacon...\n");

    send_beacon();

    printf("Listening and publishing to MQTT...\n");

    struct lgw_pkt_rx_s rxpkt[8];
    char hex_payload[512];
    
    pkt_id_t last_pkt = {0};
    
    while (!exit_sig) {

        int nb_pkt = lgw_receive(8, rxpkt);

        if (nb_pkt > 0) {
            
            // for (int i = 0; i < nb_pkt; i++) {

            //     if (rxpkt[i].status == STAT_CRC_OK) {
            //             printf("Received pkt on freq: %d, offset: %d, if chain: %d, modem id: %d, count_us: %d", rxpkt[i].freq_hz, rxpkt[i].freq_offset, rxpkt[i].if_chain, rxpkt[i].modem_id, rxpkt[i].count_us);
            //             memset(hex_payload, 0, sizeof(hex_payload));
            //             payload_to_hex(rxpkt[i].payload, rxpkt[i].size, hex_payload);

            //             mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(hex_payload), hex_payload, 0, false);

            //             printf("Published: %s\n", hex_payload);

                    
            //     }
            //}

            for (int i = 0; i < nb_pkt; i++) {
                if (rxpkt[i].status == STAT_CRC_OK) {

                    pkt_id_t cur_pkt = {rxpkt[i].freq_hz, rxpkt[i].freq_offset, rxpkt[i].count_us, rxpkt[i].crc};

                    // ignora se mesmo modem + IF + timestamp
                    if (!(cur_pkt.freq_hz == last_pkt.freq_hz &&
                        cur_pkt.freq_offset == last_pkt.freq_offset &&
                        cur_pkt.crc == last_pkt.crc &&
                        cur_pkt.count_us == last_pkt.count_us) &&
                        rxpkt[i].payload[0] == 0x02) {

                        last_pkt = cur_pkt;

                        memset(hex_payload, 0, sizeof(hex_payload));
                        payload_to_hex(rxpkt[i].payload, rxpkt[i].size, hex_payload);
                        mosquitto_publish(mosq, NULL, MQTT_TOPIC, rxpkt[i].size, rxpkt[i].payload, 0, false);

                        fprintf(csv_file, "%lu,%s\n", (unsigned long)time(NULL), hex_payload);
                        fflush(csv_file); // make sure it's written immediately

                        printf("Received pkt with rssi: %.2f, snr: %.2f, on freq: %d, offset: %d, if chain: %d, modem id: %d, count_us: %d\nPublished: %s\n",
                            rxpkt[i].rssis, rxpkt[i].snr, rxpkt[i].freq_hz, rxpkt[i].freq_offset, rxpkt[i].if_chain, rxpkt[i].modem_id, rxpkt[i].count_us, hex_payload);
                    }
                }
            }
            
        }

        mosquitto_loop(mosq, 0, 1);
        wait_ms(10);
    }

    lgw_stop();
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();


    fclose(csv_file);
    return 0;
}
