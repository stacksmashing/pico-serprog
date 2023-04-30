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
#include "pio/pio_spi.h"
#include "spi.h"

#define PIN_MISO 4
#define PIN_MOSI 3
#define PIN_SCK 2
#define PIN_CS 1
#define BUS_SPI         (1 << 3)
#define S_SUPPORTED_BUS   BUS_SPI
#define S_CMD_MAP ( \
  (1 << S_CMD_NOP)       | \
  (1 << S_CMD_Q_IFACE)   | \
  (1 << S_CMD_Q_CMDMAP)  | \
  (1 << S_CMD_Q_PGMNAME) | \
  (1 << S_CMD_Q_SERBUF)  | \
  (1 << S_CMD_Q_BUSTYPE) | \
  (1 << S_CMD_SYNCNOP)   | \
  (1 << S_CMD_O_SPIOP)   | \
  (1 << S_CMD_S_BUSTYPE) | \
  (1 << S_CMD_S_SPI_FREQ)| \
  (1 << S_CMD_S_PIN_STATE) \
)


static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
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

void process(pio_spi_inst_t *spi, int command) {
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

                cs_select(PIN_CS);
                fread(write_buffer, 1, wlen, stdin);
                pio_spi_write8_blocking(spi, write_buffer, wlen);

                putchar(S_ACK);
                char buf;
                
                for(uint32_t i = 0; i < rlen; i++)  {
                    pio_spi_read8_blocking(spi, &buf, 1);
                    putchar(buf);
                }

                
                cs_deselect(PIN_CS);
            }
            break;
        case S_CMD_S_SPI_FREQ:
            getu32();

            // TODO
            putchar(S_ACK);
            putchar(0x0);
            putchar(0x40);
            putchar(0x0);
            putchar(0x0);
            break;
        case S_CMD_S_PIN_STATE:
            //TODO:
            getchar();
            putchar(S_ACK);
            break;
        default:
            putchar(S_NAK);
    }
}

int main() {
    stdio_init_all();

    stdio_set_translate_crlf(&stdio_usb, false);


    // Initialize CS
    gpio_init(PIN_CS);
    gpio_put(PIN_CS, 1);
    gpio_set_dir(PIN_CS, GPIO_OUT);


    // We use PIO 1
    pio_spi_inst_t spi = {
            .pio = pio1,
            .sm = 0,
            .cs_pin = PIN_CS
    };

    uint offset = pio_add_program(spi.pio, &spi_cpha0_program);

    pio_spi_init(spi.pio, spi.sm, offset,
                 8,       // 8 bits per SPI frame
                 31.25f,  // 1 MHz @ 125 clk_sys
                 false,   // CPHA = 0
                 false,   // CPOL = 0
                 PIN_SCK,
                 PIN_MOSI,
                 PIN_MISO);

    // Command handling
    while(1) {
        int command = getchar();
        process(&spi, command);
    }

    return 0;
}
