/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pio_spi.h"

void pio_spi_init(pio_spi_inst_t *spi, uint n_bits, float clkdiv, bool cpha, bool cpol) {
  spi->prog_offs  = pio_add_program(spi->pio, cpha ? &spi_cpha1_program : &spi_cpha0_program);
    pio_sm_config c = cpha ? spi_cpha1_program_get_default_config(spi->prog_offs) : spi_cpha0_program_get_default_config(spi->prog_offs);
    sm_config_set_out_pins(&c, spi->pin_mosi, 1);
    sm_config_set_in_pins(&c, spi->pin_miso);
    sm_config_set_sideset_pins(&c, spi->pin_sck);
    // Only support MSB-first in this example code (shift to left, auto push/pull, threshold=nbits)
    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);

    // MOSI, SCK output are low, MISO is input
    pio_sm_set_pins_with_mask(spi->pio, spi->sm, 0, (1u << spi->pin_sck) | (1u << spi->pin_mosi));
    pio_sm_set_pindirs_with_mask(spi->pio, spi->sm, (1u << spi->pin_sck) | (1u << spi->pin_mosi), (1u << spi->pin_sck) | (1u << spi->pin_mosi) | (1u << spi->pin_miso));
    pio_gpio_init(spi->pio, spi->pin_mosi);
    pio_gpio_init(spi->pio, spi->pin_miso);
    pio_gpio_init(spi->pio, spi->pin_sck);

    // The pin muxes can be configured to invert the output (among other things
    // and this is a cheesy way to get CPOL=1
    gpio_set_outover(spi->pin_sck, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
    // SPI is synchronous, so bypass input synchroniser to reduce input delay.
    hw_set_bits(&spi->pio->input_sync_bypass, 1u << spi->pin_miso);

    pio_sm_init(spi->pio, spi->sm, spi->prog_offs, &c);
    pio_sm_set_enabled(spi->pio, spi->sm, true);
}

// Just 8 bit functions provided here. The PIO program supports any frame size
// 1...32, but the software to do the necessary FIFO shuffling is left as an
// exercise for the reader :)
//
// Likewise we only provide MSB-first here. To do LSB-first, you need to
// - Do shifts when reading from the FIFO, for general case n != 8, 16, 32
// - Do a narrow read at a one halfword or 3 byte offset for n == 16, 8
// in order to get the read data correctly justified. 

void __time_critical_func(pio_spi_write8_blocking)(const pio_spi_inst_t *spi, const uint8_t *src, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    // Do 8 bit accesses on FIFO, so that write data is byte-replicated. This
    // gets us the left-justification for free (for MSB-first shift-out)
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            (void) *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *dst, size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = 0;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

void __time_critical_func(pio_spi_write8_read8_blocking)(const pio_spi_inst_t *spi, uint8_t *src, uint8_t *dst,
                                                         size_t len) {
    size_t tx_remain = len, rx_remain = len;
    io_rw_8 *txfifo = (io_rw_8 *) &spi->pio->txf[spi->sm];
    io_rw_8 *rxfifo = (io_rw_8 *) &spi->pio->rxf[spi->sm];
    while (tx_remain || rx_remain) {
        if (tx_remain && !pio_sm_is_tx_fifo_full(spi->pio, spi->sm)) {
            *txfifo = *src++;
            --tx_remain;
        }
        if (rx_remain && !pio_sm_is_rx_fifo_empty(spi->pio, spi->sm)) {
            *dst++ = *rxfifo;
            --rx_remain;
        }
    }
}

