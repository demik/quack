set(srcs "adb.c"
         "blue.c"
         "esp_hid_gap.c"
	 "gpio.c"
	 "led.c"
	 "main.c"
	 "quad.c"
	 "wii.c")

set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES driver esp_hid nvs_flash spi_flash)
