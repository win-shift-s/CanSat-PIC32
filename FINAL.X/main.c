#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "platform.h"

// ESP32
#define UART (&(SERCOM0_REGS->USART_INT))
bool co2_flag = false;
bool pms_flag = false;
bool gps_flag = false;
char buffer[256];

// MH-Z19C
//#define MHZ19C_START 0xFF
//#define MHZ19C_COMMAND 0x86
//#define MHZ19C_CHECKSUM 0x79
#define CO2_BUF_SIZE 9
static char co2_buf[CO2_BUF_SIZE];
static size_t co2_idx = 0;

// PMS5003T
#define PMS_START_1 0x42
#define PMS_START_2 0x4D
#define PMS_BUF_SIZE 32
static char pms_buf[PMS_BUF_SIZE];
static size_t pms_idx = 0;

// NEO-6M
#define GPS_BUF_SIZE 128  // Buffer size for storing NMEA sentence
static char gps_buf[GPS_BUF_SIZE];  // Buffer for storing the sentence
static size_t gps_idx = 0;        // Index to keep track of buffer position
static bool gps_sentence_flag = false;   // Flag to indicate sentence is ready
static char latBuffer[GPS_BUF_SIZE];  // Buffer for latitude in NMEA
static char longBuffer[GPS_BUF_SIZE];  // Buffer for longitude in NMEA

//static const char banner_msg[] =
//"\033[0m\033[2J\033[1;1H"
//"+-----------------------------------------------------------+\r\n"
//"| Air Quality Sensor Readings                               |\r\n"
//"+-----------------------------------------------------------+\r\n";
//
//static const char ESC_SEQ_GPS_LINE[] = "\033[11;1H\033[0K";

typedef struct prog_state_type {
    uint16_t flags;
//    
    platform_usart_tx_bufdesc_t esp_tx_desc[4];
    char esp_tx_buf[128];

    platform_usart_tx_bufdesc_t co2_tx_desc;
    char co2_tx_buf[CO2_BUF_SIZE];

    platform_usart_rx_async_desc_t esp_rx_desc;
    char esp_rx_buf[128];

    platform_usart_rx_async_desc_t co2_rx_desc;
    char co2_rx_buf[CO2_BUF_SIZE];

    platform_usart_rx_async_desc_t pms_rx_desc;
    char pms_rx_buf[PMS_BUF_SIZE];

    platform_usart_rx_async_desc_t gps_rx_desc;
    char gps_rx_buf[GPS_BUF_SIZE];

} prog_state_t;

static void prog_setup(prog_state_t *ps) {
    platform_init();

    // Send the banner message
//    ps->esp_tx_desc[0].buf = banner_msg;
//    ps->esp_tx_desc[0].len = sizeof(banner_msg) - 1;
//    platform_usart_esp_tx_async(&ps->esp_tx_desc[0], 1);

    ps->co2_rx_desc.buf = ps->co2_rx_buf;
    ps->co2_rx_desc.max_len = 9;
    platform_usart_co2_rx_async(&ps->co2_rx_desc);

    ps->pms_rx_desc.buf = ps->pms_rx_buf;
    ps->pms_rx_desc.max_len = 32;
    platform_usart_pms_rx_async(&ps->pms_rx_desc);

    ps->gps_rx_desc.buf = ps->gps_rx_buf;
    ps->gps_rx_desc.max_len = sizeof(ps->gps_rx_buf);
    platform_usart_gps_rx_async(&ps->gps_rx_desc);
}

int read_count(){
    TCC1_REGS->TCC_CTRLBSET = (0x80);
    return TCC1_REGS->TCC_COUNT; // Return back the counter value 
}

static void CO2_Read(prog_state_t *ps) {
    // Send to MH-Z19C
    if (!platform_usart_co2_tx_busy()) {
        static const uint8_t cmd[8] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00};
        memcpy(ps->co2_tx_buf, cmd, 8);
        uint8_t checksum = 0;
        for (int i = 1; i < 8; i++) {
            checksum += ps->co2_tx_buf[i];
        }
        ps->co2_tx_buf[8] = 0xFF - checksum + 0x1;
        ps->co2_tx_desc.buf = ps->co2_tx_buf;
        ps->co2_tx_desc.len = 9;
        platform_usart_co2_tx_async(&ps->co2_tx_desc, 1);
    }
    delay(1000);

    if (!platform_usart_co2_rx_busy()) {
        char *rx = (uint8_t *)ps->co2_rx_buf[0];
        char debug[128];

        if (co2_idx < GPS_BUF_SIZE - 1) {
            co2_buf[co2_idx++] = data;
        }
        // Print raw RX for debugging
        int pos = 0;
        pos += sprintf(&debug[pos], "MH-Z19C RX: ");
        for (int i = 0; i < 9; ++i) {
            pos += sprintf(&debug[pos], "%02X ", rx[i]);
        }
        pos += sprintf(&debug[pos], "\r\n");

        // Check header
        if (rx[0] == 0xFF && rx[1] == 0x86) {
            // Verify checksum
            uint8_t sum = 0;
            for (int i = 1; i < 8; ++i) sum += rx[i];
            uint8_t expected = 0xFF - sum + 1;

            uint16_t co2 = (rx[2] << 8) | rx[3];
            if (expected == rx[8]) {
                pos += sprintf(&debug[pos], "CO2: %u ppm\r\n", co2);
            } else {
                pos += sprintf(&debug[pos], "Checksum error (expected %02X, got %02X)\r\n", expected, rx[8]);
                pos += sprintf(&debug[pos], "CO2: %u ppm\r\n", co2);
            }
        } else {
            pos += sprintf(&debug[pos], "Invalid MH-Z19C header\r\n");
        }

        // Send to CDC serial output
        ps->esp_tx_desc[0].buf = debug;
        ps->esp_tx_desc[0].len = strlen(debug);
        platform_usart_esp_tx_async(&ps->esp_tx_desc[0], 1);

        memset(ps->co2_rx_buf, 0, sizeof(ps->co2_rx_buf)); // Clear buffer before next read
        platform_usart_co2_rx_async(&ps->co2_rx_desc);
    }
}

static void PMS_Read(prog_state_t *ps) {
    if (!platform_usart_pms_rx_busy()) {
        uint8_t *data = (uint8_t *)ps->pms_rx_buf;

        if (data[0] == PMS_START_1 && data[1] == PMS_START_2) {  
            pms_buf[0] = data[12];
            pms_buf[1] = data[13];
            pms_buf[2] = data[14];
            pms_buf[3] = data[15];
            pms_buf[4] = data[24];
            pms_buf[5] = data[25];
            pms_buf[6] = data[26];
            pms_buf[7] = data[27];
        }

        // Send to ESP8266
        ps->esp_tx_desc[2].buf = pms_buf;
        ps->esp_tx_desc[2].len = 8;
        platform_usart_esp_tx_async(&ps->esp_tx_desc[2], 1);

        // Restart reception
        memset(ps->pms_rx_buf, 0, sizeof(ps->pms_rx_buf)); // Clear buffer before next read
        platform_usart_pms_rx_async(&ps->pms_rx_desc);
    }
}

//static void PMS_Read(prog_state_t *ps) {
//    if (!platform_usart_pms_rx_busy()) {
//        uint8_t data = ps->pms_rx_buf;
//
//        if (data[0] == PMS_START_1 && data[1] == PMS_START_2) {
//            uint16_t pm2_5 = ((uint16_t)data[12] << 8) | data[13];
//            uint16_t pm10  = ((uint16_t)data[14] << 8) | data[15];
//            int16_t  temp  = ((int16_t)data[24] << 8) | data[25];
//            uint16_t rhum  = ((uint16_t)data[26] << 8) | data[27];
//
//            // Format: "PM25=123;PM10=456;T=25.3;RH=45.6\n"
//            // Note: temp and rhum are sent as integers ×10, divide to get decimal
//            snprintf(pms_buf, sizeof(pms_buf),
//                     "PM25=%u;PM10=%u;T=%i;RH=%u\n",
//                     pm2_5,
//                     pm10,
//                     temp / 10,
//                     rhum / 10);
//
//            ps->esp_tx_desc[0].buf = pms_buf;
//            ps->esp_tx_desc[0].len = strnlen(pms_buf, sizeof(pms_buf));
//            platform_usart_esp_tx_async(&ps->esp_tx_desc[0], 1);
//        }
//
//        memset(ps->pms_rx_buf, 0, sizeof(ps->pms_rx_buf));
//        platform_usart_pms_rx_async(&ps->pms_rx_desc);
//    }
//}

static void GPS_Read(prog_state_t *ps) {
    if (!platform_usart_gps_rx_busy()) {
        // Store received byte in the buffer
        char data = ps->gps_rx_buf[0];

        // Store received byte in gpsBuffer if there's space
        if (gps_idx < GPS_BUF_SIZE - 1) {
            gps_buf[gps_idx++] = data;

            // Check if sentence is complete (NMEA sentences end with '\n')
            if (data == '\n') {
                gps_buf[gps_idx] = '\0';  // Null-terminate the string
                gps_sentence_flag = true;        // Mark sentence as ready
//                gps_idx = 0;                // Reset buffer for the next sentence
            }
        } else {
            // Buffer overflow: Reset buffer
            gps_idx = 0;
        }

        // Check if a complete GPGGA sentence is received
        if (gps_sentence_flag) {
            // Check if the sentence contains "$GPGGA"
            if (strstr(gps_buf, "$GPGGA") != NULL) {
//                memcpy(ps->esp_tx_buf, gps_buf, strnlen(gps_buf, GPS_BUF_SIZE));
                strcat(buffer, gps_buf);
                // Send to ESP8266
                ps->esp_tx_desc[0].buf = buffer;
                ps->esp_tx_desc[0].len = strnlen(buffer, GPS_BUF_SIZE);
                platform_usart_esp_tx_async(&ps->esp_tx_desc[0], 1);
//                gps_flag = true;
            }
        // Clear the buffer and reset flags for the next sentence
            memset(gps_buf, 0, sizeof(gps_buf));
            gps_sentence_flag = false;
            gps_idx = 0;
        }

        // Restart reception for the next byte
        memset(ps->gps_rx_buf, 0, sizeof(ps->gps_rx_buf));
        platform_usart_gps_rx_async(&ps->gps_rx_desc);
    }
}

// USART1 = transmitter
char receiveChar(){
    while ((UART->SERCOM_INTFLAG & (1 << 2)) == 0);
    return (char)(UART->SERCOM_DATA);
}

void write(char data) {
    while ((UART->SERCOM_INTFLAG & (1 << 0)) == 0);
    UART->SERCOM_DATA = data;
}

void sendString(const char* str) {
    while (*str) {
       write(*str++);
    }   
    crude_delay_ms(1000);
}

static void prog_loop_one(prog_state_t *ps) {
    platform_do_loop_one();

//    if (read_count() < 1000) {
//        while(read_count() < 1000);
        GPS_Read(ps);
//    }

//    if (read_count() < 2000) {
//        while(read_count() < 2000);
//        PMS_Read(ps);
//    }

//    if (read_count() < 3000) {
//        while(read_count() < 3000);
//        CO2_Read(ps);
//    }
}

int main(void) {
    prog_state_t ps;

    // Initialization time
    prog_setup(&ps);

    // Infinite loop
    for (;;) {
        prog_loop_one(&ps);
        if (co2_flag == true && pms_flag == true && gps_flag == true){
//            strcat(buffer, co2_buf);
//            strcat(buffer, pms_buf);
            strcat(buffer, gps_buf);
        }
//        sendString("Hello World!");
    }
    return 1;
}