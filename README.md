# ZC2X

Open Source Embedded Streaming Platform

Inspired by C-V2X

Powered by ESP-IDF

Part of the ZC8 Ecosystem.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the system overview
(data path, firmware roles, packet format) and [docs/README.md](docs/README.md)
for the full documentation index.

---

## Build & flash

Load the ESP-IDF environment:

```sh
cd ~/.espressif/v6.0.2/esp-idf/
./install.sh
source export.sh
```

Go to a firmware project (`obu` and `rsu` target `esp32c6`; `ecu` targets `esp32s3`):

```sh
cd platform/esp-idf/firmware/obu
cd platform/esp-idf/firmware/rsu
cd platform/esp-idf/firmware/ecu
```

Set the target:

```sh
idf.py set-target esp32c6   # obu, rsu
idf.py set-target esp32s3   # ecu
```

Build:

```sh
idf.py build
```

Flash:

```sh
idf.py -p /dev/tty.usbmodem1101 flash
```

Flash and monitor:

```sh
idf.py -p /dev/tty.usbmodem1101 flash monitor
```

## Shared components

`framework/core/packet` and `framework/core/types` are implemented and
consumed by all three firmware targets. Everything else under `framework/`
(`config`, `logger`, `bus`, `time`, `hal`, `protocols`, `services`,
`transports`, `utilities`) is scaffolded but not yet implemented — see
[docs/ARCHITECTURE.md §6](docs/ARCHITECTURE.md#6-implemented-vs-planned).

## Configuration convention (aspirational)

This is a rule established for the whole project — not yet followed by any
current firmware, which reads `device_config.h` macros directly in `main.c`:

```
device_config.h
    │
    ▼
  main.c
    │
    ▼
zc2x_config_init(...)
    │
    ▼
zc2x_config
    │
    ▼
ALL OTHER COMPONENTS
```

Only `main.c` should include `device_config.h`. Every other component should
obtain configuration through:

```c
const zc2x_config_t *cfg = zc2x_config_get();
```

This gives a single centralized runtime configuration object and prevents
compile-time macros from leaking throughout the codebase. No component
should include `device_config.h` directly.
