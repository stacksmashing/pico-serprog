/**
 * Written by Thomas Roth - code@stacksmashing.net
 * 
 * Licensed under GPLv3
 * 
 * Based on the spi_flash pico-example, which is:
 *  Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * Also based on stm32-vserprog: 
 *  https://github.com/dword1511/stm32-vserprog
 * 
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "pio/pio_spi.h"
#include "spi.h"

#define PIN_LED PICO_DEFAULT_LED_PIN
#define PIN_MISO 4
#define PIN_MOSI 3
#define PIN_SCK 2
#define PIN_CS_0 1
#define PIN_CS_1 5
#define PIN_CS_2 6
#define PIN_CS_3 7
#define BUS_SPI         (1 << 3)
#define S_SUPPORTED_BUS   BUS_SPI
#define S_CMD_MAP ( \
  (1 << S_CMD_NOP)         | \
  (1 << S_CMD_Q_IFACE)     | \
  (1 << S_CMD_Q_CMDMAP)    | \
  (1 << S_CMD_Q_PGMNAME)   | \
  (1 << S_CMD_Q_SERBUF)    | \
  (1 << S_CMD_Q_BUSTYPE)   | \
  (1 << S_CMD_SYNCNOP)     | \
  (1 << S_CMD_O_SPIOP)     | \
  (1 << S_CMD_S_BUSTYPE)   | \
  (1 << S_CMD_S_SPI_FREQ)  | \
  (1 << S_CMD_S_PIN_STATE) | \
  (1 << S_CMD_S_SPI_CS)      \
)

uint active_cs_pin = 0;
#define NUM_CS_AVAILABLE 4 // Number of usable chip selects
uint8_t cs_pins[NUM_CS_AVAILABLE] = { PIN_CS_0, PIN_CS_1, PIN_CS_2, PIN_CS_3 };

static uint32_t serprog_spi_init(uint32_t freq);

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pins[cs_pin], 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pins[cs_pin], 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

uint32_t getu24() {
    uint32_t c1 = getchar();
    uint32_t c2 = getchar();
    uint32_t c3 = getchar();
    return c1 | (c2<<8) | (c3<<16);
}

uint32_t getu32() {
    uint32_t c1 = getchar();
    uint32_t c2 = getchar();
    uint32_t c3 = getchar();
    uint32_t c4 = getchar();
    return c1 | (c2<<8) | (c3<<16) | (c4<<24);
}

void putu32(uint32_t d) {
    char buf[4];
    memcpy(buf, &d, 4);
    putchar(buf[0]);
    putchar(buf[1]);
    putchar(buf[2]);
    putchar(buf[3]);
}

unsigned char write_buffer[4096];

void apply_pin_state(const pio_spi_inst_t *spi, bool state) {
    pio_spi_enable_outputs(spi->pio, spi->sm, state, PIN_SCK, PIN_MOSI, PIN_MISO);
    for (int i = 0; i < NUM_CS_AVAILABLE; i++) {
        gpio_set_dir(cs_pins[i], state? GPIO_OUT : GPIO_IN);
    }
}

void process(const pio_spi_inst_t *spi, int command) {
    static bool pin_state = false;

    switch(command) {
        case S_CMD_NOP:
            putchar(S_ACK);
            break;
        case S_CMD_Q_IFACE:
            putchar(S_ACK);
            putchar(0x01);
            putchar(0x00);
            break;
        case S_CMD_Q_CMDMAP:
            putchar(S_ACK);
            putu32(S_CMD_MAP);

            for(int i = 0; i < 32 - sizeof(uint32_t); i++) {
                putchar(0);
            }
            break;
        case S_CMD_Q_PGMNAME:
            putchar(S_ACK);
            fwrite("pico-serprog\x0\x0\x0\x0\x0", 1, 16, stdout);
            fflush(stdout);
            break;
        case S_CMD_Q_SERBUF:
            putchar(S_ACK);
            putchar(0xFF);
            putchar(0xFF);
            break;
        case S_CMD_Q_BUSTYPE:
            putchar(S_ACK);
            putchar(S_SUPPORTED_BUS);
            break;
        case S_CMD_SYNCNOP:
            putchar(S_NAK);
            putchar(S_ACK);
            break;
        case S_CMD_S_BUSTYPE:
            {
                int bustype = getchar();
                if((bustype | S_SUPPORTED_BUS) == S_SUPPORTED_BUS) {
                    putchar(S_ACK);
                } else {
                    putchar(S_NAK);
                }
            }
            break;
        case S_CMD_O_SPIOP:
            {

                uint32_t wlen = getu24();
                uint32_t rlen = getu24();

                cs_select(active_cs_pin);
                fread(write_buffer, 1, wlen, stdin);
                pio_spi_write8_blocking(spi, write_buffer, wlen);

                putchar(S_ACK);
                uint32_t chunk;
                char buf[128];

                for(uint32_t i = 0; i < rlen; i += chunk) {
                    chunk = MIN(rlen - i, sizeof(buf));
                    pio_spi_read8_blocking(spi, buf, chunk);
                    fwrite(buf, 1, chunk, stdout);
                    fflush(stdout);
                }

                cs_deselect(active_cs_pin);
            }
            break;
        case S_CMD_S_SPI_FREQ:
            {
                uint32_t freq = getu32();
                if (freq >= 1) {
                    putchar(S_ACK);
                    putu32(serprog_spi_init(freq));
                    apply_pin_state(spi, pin_state);
                } else {
                    putchar(S_NAK);
                }
            }
            break;
        case S_CMD_S_PIN_STATE:
            pin_state = !!getchar();
            apply_pin_state(spi, pin_state);
            putchar(S_ACK);
            break;
        case S_CMD_S_SPI_CS:
            {
                uint8_t new_cs = getchar();
                if (new_cs < NUM_CS_AVAILABLE) {
                        active_cs_pin = new_cs;
                        putchar(S_ACK);
                } else {
                        putchar(S_NAK);
                }
                break;
            }
        default:
            putchar(S_NAK);
    }
}

// We use PIO 1
static const pio_spi_inst_t spi = {
    .pio = pio1,
    .sm = 0,
};
static uint spi_offset;

static inline float freq_to_clkdiv(uint32_t freq) {
    float div = clock_get_hz(clk_sys) * 1.0 / (freq * pio_spi_cycles_per_bit);

    if (div < 1.0)
        div = 1.0;
    if (div > 65536.0)
        div = 65536.0;

    return div;
}

static inline uint32_t clkdiv_to_freq(float div) {
    return clock_get_hz(clk_sys) / (div * pio_spi_cycles_per_bit);
}

static uint32_t serprog_spi_init(uint32_t freq) {

    float clkdiv = freq_to_clkdiv(freq);

    pio_spi_init(spi.pio, spi.sm, spi_offset,
                 8,       // 8 bits per SPI frame
                 clkdiv,
                 false,   // CPHA = 0
                 false,   // CPOL = 0
                 PIN_SCK,
                 PIN_MOSI,
                 PIN_MISO);

    return clkdiv_to_freq(clkdiv);
}

int main() {
    // Metadata for picotool
    bi_decl(bi_program_description("Flashrom/serprog compatible firmware for the Raspberry Pi Pico"));
    bi_decl(bi_program_url("https://github.com/stacksmashing/pico-serprog"));
    bi_decl(bi_1pin_with_name(PIN_LED, "LED"));
    bi_decl(bi_1pin_with_name(PIN_MISO, "MISO"));
    bi_decl(bi_1pin_with_name(PIN_MOSI, "MOSI"));
    bi_decl(bi_1pin_with_name(PIN_SCK, "SCK"));
    bi_decl(bi_1pin_with_name(PIN_CS_0, "CS_0 (default)"));
    bi_decl(bi_1pin_with_name(PIN_CS_1, "CS_1"));
    bi_decl(bi_1pin_with_name(PIN_CS_2, "CS_2"));
    bi_decl(bi_1pin_with_name(PIN_CS_3, "CS_3"));

    stdio_init_all();

    stdio_set_translate_crlf(&stdio_usb, false);


    // Initialize CS
    for (int i = 0; i < NUM_CS_AVAILABLE; i++) {
        gpio_init(cs_pins[i]);
        gpio_put(cs_pins[i], 1);
        gpio_set_dir(cs_pins[i], GPIO_IN); // switch to output on S_CMD_S_PIN_STATE
    }

    spi_offset = pio_add_program(spi.pio, &spi_cpha0_program);
    serprog_spi_init(1000000); // 1 MHz

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Command handling
    while(1) {
        int command = getchar();

        gpio_put(PIN_LED, 1);
        process(&spi, command);
        gpio_put(PIN_LED, 0);
    }

    return 0;
}
