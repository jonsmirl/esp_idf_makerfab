idf_component_register(SRCS "lcd_example_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_lcd esp_timer esp_lcd_touch_ft5x06 esp_lcd_ili9488
                       )
