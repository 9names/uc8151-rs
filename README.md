# uc8151-rs - a no-std Rust library for the UC8151(IL0373) e-ink display
This is a Rust port of the Pimoroni UC8151 library (this controller is also known as IL0373).

The current implementation only supports the particular variant used on the Pimoroni badger2040.
badger2040 uses a black and white (1bit per pixel) 128x296 pixel screen.
It currently does not support partial updates.

It will support [embedded-graphics](https://github.com/embedded-graphics/embedded-graphics) eventually.
