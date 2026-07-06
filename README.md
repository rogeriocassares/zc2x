Go to Project:

Load the idf.py:

```sh
cd ~/.espressif/v6.0.2/esp-idf/
./install.sh
source export.sh
```

Go to project:

```sh
cd ./zc2x/firmware/rsu
cd ./zc2x/firmware/obu
```

Set the target:

```sh
idf.py set-target esp32c6
```

Build:

```sh
idf.py build
```

Flash:

```sh
idf.py -p /dev/tty.usbmodem1101 flash
```

Flash and Monitor:

```sh
idf.py -p /dev/tty.usbmodem1101 flash monitor
```

zc2x_logger zc2x_config zc2x_event zc2x_bus zc2x_storage zc2x_can zc2x_gnss zc2x_wifi zc2x_http zc2x_nats zc2x_xbee zc2x_dashboard zc2x_ota

6. Do any components include device_config.h?

No.

This is a rule I'd establish for the whole project:

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

Only main.c should include device_config.h.

Every other component should obtain configuration through:

const zc2x_config_t \*cfg = zc2x_config_get();

This gives you one centralized runtime configuration object and prevents compile-time macros from leaking throughout the codebase.
