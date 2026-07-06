Go to Project:

Load the idf.py:

```sh
cd ~/.espressif/v6.0.2/esp-idf/
./install.sh
source export.sh
```

Go to project:

```sh
cd ./zc2x/projects/rsu
cd ./zc2x/projects/obu
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
