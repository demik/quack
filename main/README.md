# Quack Firmware source code

### Supported SDK versions:

The code is designed to be build with ESP-IDF SDK version 4.2.3. To workaround some patchs or limitations of this specific release, you need to apply the patch located in the SDK subfolder

### Unsupported mouse conversions:

Functions are grouped into "domains" in different files. Each functions has a prefix for its own domain.
For example, adb_init() is the init function for the ADB Bus and is located in the adb.c source file.

List of relevant files:
- adb.* Apple Desktop Bus related functions
- blue.* Bluetooth related functions
- esp_hid_gap.* HID GAP library from Espressif
- gpio.* raw GPIO related functions
- led.* LED related functions
- m4848.h Header for Apple Hockey Puck mouse over Bluetooth emulation
- main.c Everything start here
- quad.c Quadrature related functions (mouse port)
- wii.* Wiimote driver
