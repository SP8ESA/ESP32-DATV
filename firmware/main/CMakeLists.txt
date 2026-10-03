idf_component_register(SRCS "main.c" "lut8.S"
                       REQUIRES esp_phy esp_wifi nvs_flash esp_timer esp_driver_usb_serial_jtag)
