# pico-serprog

This is a basic flashrom/serprog compatible SPI flash reader/writer for the Raspberry Pi Pico.

It does not require a custom version of flashrom, just drag the compiled uf2 onto your Pico and you're ready to go.

The default pin-out is:

| GPIO | Pico Pin | Function       |
|------|----------|----------------|
| 1    |    2     | CS_0 (default) |
| 2    |    4     | SCK            |
| 3    |    5     | MOSI           |
| 4    |    6     | MISO           |
| 5    |    7     | CS_1           |
| 6    |    9     | CS_2           |
| 7    |    10    | CS_3           |

## Usage

Dump a flashchip:

```
flashrom -p serprog:dev=/dev/ttyACM0:115200,spispeed=12M -r foo.bin
```

pico-serprog only switches the pins to output when requested by flashrom. This
means that you can leave your pico-serprog programmer attached to the flash;
you don't have to detach it before booting the board that you're programming.


## License

The project is based on the spi_flash example by Raspberry Pi (Trading) Ltd. which is licensed under BSD-3-Clause.

As a lot of the code itself was heavily inspired/influenced by `stm32-vserprog` this code is licensed under GPLv3.
