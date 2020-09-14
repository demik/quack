set(srcs "blue.c"
         "esp_hid_gap.c"
	 "gpio.c"
	 "main.c")
         
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid)
