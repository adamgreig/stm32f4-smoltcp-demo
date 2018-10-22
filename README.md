# Example Application for stm32f4-smoltcp

This crate contains a simple example application for the [stm32f4-smoltcp]
ethernet driver.

[stm32f4-smoltcp]: https://github.com/adamgreig/stm32f4-smoltcp

You'll likely need to modify `gpio_init` for your particular board.

Once running, the board will respond to ping on `192.168.2.101` and telnet on
port 23.
