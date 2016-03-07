
# 0.1.4

- reintroduced swd.cpp for Particle Build library import

# 0.1.3

## Bugfixes

- Consume all input after the initial keypress. (Fixes issue on Windows where the pressing enter sends CR+LF, but only one character was consumed.)


# 0.1.2

## Features

- Change the LED to Blue on startup, and then Green on success.

## Bugfixes

- fixed incorrect bootloader on P1/Electron [#1](https://github.com/m-mcgowan/embedded-swd/issues/1)
- fixed runaway loop when device not detected
