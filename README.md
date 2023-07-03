# pico-serprog

This is a very basic flashrom/serprog compatible SPI flash reader/writer for the Raspberry Pi Pico.

It does not require a custom version of flashrom, just drag the compiled uf2 onto your Pico and you're ready to go.

The default pin-out is:

| GPIO | Pico Pin | Function |
|------|----------|----------|
| 1    |    2     | CS       |
| 2    |    4     | SCK      |
| 3    |    5     | MOSI     |
| 4    |    6     | MISO     |
| 5    |    7     | CRESET   |

## Usage

Dump a flashchip:

```
flashrom -p serprog:dev=/dev/ttyACM0:115200 -r foo.bin
```

## License

The project is based on the spi_flash example by Raspberry Pi (Trading) Ltd. which is licensed under BSD-3-Clause.

As a lot of the code itself was heavily inspired/influenced by `stm32-vserprog` this code is licensed under GPLv3.
