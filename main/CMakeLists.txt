set(srcs "adb.c" 
         "blue.c"
         "esp_hid_gap.c"
	 "gpio.c"
	 "led.c"
	 "main.c"
	 "quad.c")
         
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid)
