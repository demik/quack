set(srcs "adb.c"
         "blue.c"
         "esp_hid_gap.c"
         "gpio.c"
         "led.c"
         "main.c"
         "mighty.c"
         "phy.c"
         "quad.c"
         "wii.c")

set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES bt esp_driver_ledc esp_driver_gpio esp_hid esp_ringbuf nvs_flash spi_flash)
