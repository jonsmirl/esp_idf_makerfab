/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include <esp_lcd_ili9488.h>
#include "esp_lcd_touch_ft5x06.h"
#include "demos/lv_demos.h"

static const char *TAG = "example";
static lv_disp_drv_t disp_drv;

#define CONFIG_I2C_MASTER_SCL 39
#define CONFIG_I2C_MASTER_SDA 38

// I2C settings
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*Read the touchpad*/
void example_touchpad_read( lv_indev_drv_t * drv, lv_indev_data_t * data )
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        ESP_LOGI(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ      (40 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL   (1)
#define LCD_BK_LIGHT_OFF_LEVEL  (!EXAMPLE_LCD_BK_LIGHT_ON_LEVEL)

#define PIN_NUM_LCD_RD          (GPIO_NUM_48)
#define PIN_NUM_LCD_WR          (GPIO_NUM_35)
#define PIN_NUM_LCD_CS          (GPIO_NUM_37)
#define PIN_NUM_LCD_DC          (GPIO_NUM_36)
#define PIN_NUM_LCD_DATA0       (GPIO_NUM_47)  //B
#define PIN_NUM_LCD_DATA1       (GPIO_NUM_21)  //B
#define PIN_NUM_LCD_DATA2       (GPIO_NUM_14)  //B
#define PIN_NUM_LCD_DATA3       (GPIO_NUM_13)  //B
#define PIN_NUM_LCD_DATA4       (GPIO_NUM_12)  //B
#define PIN_NUM_LCD_DATA5       (GPIO_NUM_11)  //G
#define PIN_NUM_LCD_DATA6       (GPIO_NUM_10)  //G
#define PIN_NUM_LCD_DATA7       (GPIO_NUM_9)  //G
#define PIN_NUM_LCD_DATA8       (GPIO_NUM_3)  //G
#define PIN_NUM_LCD_DATA9       (GPIO_NUM_8)  //G
#define PIN_NUM_LCD_DATA10      (GPIO_NUM_16)  //G
#define PIN_NUM_LCD_DATA11      (GPIO_NUM_15)  //R
#define PIN_NUM_LCD_DATA12      (GPIO_NUM_7)  //R
#define PIN_NUM_LCD_DATA13      (GPIO_NUM_6)  //R
#define PIN_NUM_LCD_DATA14      (GPIO_NUM_5)  //R
#define PIN_NUM_LCD_DATA15      (GPIO_NUM_4)  //R
#define PIN_NUM_LCD_RST         (GPIO_NUM_NC)
#define PIN_NUM_BK_LIGHT        (GPIO_NUM_45)

#define LCD_H_RES               (480)
#define LCD_V_RES               (320)
// Bit number used to represent command and parameter
#define LCD_CMD_BITS            (8)
#define LCD_PARAM_BITS          (8)
#define LVGL_TICK_PERIOD_MS     (2)
//static const size_t LV_BUFFER_SIZE = LCD_H_RES * 20 * 2;
static const size_t LV_BUFFER_SIZE = 0;


static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void x_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_V_RES,
        .y_max = LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = 4,
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller FT5x06");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)

    gpio_config_t rd_gpio_config = {
        .pin_bit_mask = 1ULL << PIN_NUM_LCD_RD,
        .mode = GPIO_MODE_OUTPUT
    };
    ESP_ERROR_CHECK(gpio_config(&rd_gpio_config));
    gpio_set_level(PIN_NUM_LCD_RD, 1);

    ESP_LOGI(TAG, "Initialize i80 bus");
    esp_lcd_i80_bus_config_t i80_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .wr_gpio_num = PIN_NUM_LCD_WR,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            PIN_NUM_LCD_DATA0,
            PIN_NUM_LCD_DATA1,
            PIN_NUM_LCD_DATA2,
            PIN_NUM_LCD_DATA3,
            PIN_NUM_LCD_DATA4,
            PIN_NUM_LCD_DATA5,
            PIN_NUM_LCD_DATA6,
            PIN_NUM_LCD_DATA7,
            PIN_NUM_LCD_DATA8,
            PIN_NUM_LCD_DATA9,
            PIN_NUM_LCD_DATA10,
            PIN_NUM_LCD_DATA11,
            PIN_NUM_LCD_DATA12,
            PIN_NUM_LCD_DATA13,
            PIN_NUM_LCD_DATA14,
            PIN_NUM_LCD_DATA15,
        },
        .bus_width = 16,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * 2 + 10,
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    static esp_lcd_i80_bus_handle_t i80_bus;
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&i80_config, &i80_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = (void *)&disp_drv,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
    };
    static esp_lcd_panel_io_handle_t io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ILI9488 panel driver");
    static esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_config, LV_BUFFER_SIZE, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
#else
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));
#endif

    if (PIN_NUM_BK_LIGHT != GPIO_NUM_NC) {
        ESP_LOGI(TAG, "Turn on LCD backlight");
        gpio_config_t bk_gpio_config = {
            .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
        gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
    }
    ESP_LOGI(TAG, "Free %d", (int) xPortGetFreeHeapSize());

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    //lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    //assert(buf2);
    ESP_LOGI(TAG, "Free %d", (int) xPortGetFreeHeapSize());
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LCD_H_RES * 20);
    ESP_LOGI(TAG, "Free %d", xPortGetFreeHeapSize());

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = example_touchpad_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    static const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    static esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
}

static lv_obj_t *demo_selection_panel;

typedef void (*function_pointer_t)(void);

typedef struct demo_button
{
    const char * const name;
    function_pointer_t function;
} demo_button_t;

/* Button event handler */
static void button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        function_pointer_t demo_function = (function_pointer_t)lv_event_get_user_data(e);
        if (demo_function)
        {
            lv_obj_t *label = lv_obj_get_child(btn, 0);
            ESP_LOGI(TAG, "Starting %s", lv_label_get_text(label));

#ifdef DISABLE_FLUSH_DURING_BENCHMARK
            if (demo_function == lv_demo_benchmark)
            {
                ESP_LOGI(TAG, "Starting benchmark with flush disabled. Wait a couple minutes for benchmark results. They'll be here soon.");
                disable_flush = true;
            }
#endif
            demo_function();
            lv_obj_del(demo_selection_panel);
        }
    }
}


/* List of buttons to create, and the associated demo function that needs to be called when clicked */
static demo_button_t demos[] = {
    {"Music", lv_demo_music},
    {"Widgets", lv_demo_widgets},
    {"Encoder", lv_demo_keypad_encoder},
    {"Benchmark", lv_demo_benchmark},
    {"Stress", lv_demo_stress}
};


void app_main(void) {
    x_main();

    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_hex(0x000000));

    // Create buttons to pick which demo
    demo_selection_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(demo_selection_panel, lv_pct(100), 120);
    lv_obj_set_scroll_snap_x(demo_selection_panel, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_flex_flow(demo_selection_panel, LV_FLEX_FLOW_ROW);
    lv_obj_align(demo_selection_panel, LV_ALIGN_CENTER, 0, 20);
    lv_obj_add_style(demo_selection_panel, &style, 0);

    for(int i = 0; i < 5; i++)
    {
        lv_obj_t * btn = lv_btn_create(demo_selection_panel);
        lv_obj_set_size(btn, 120, lv_pct(100));

        lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_CLICKED, (void*)demos[i].function);

        lv_obj_t * label = lv_label_create(btn);
        lv_label_set_text_static(label, demos[i].name);

        lv_obj_center(label);
    }
    lv_obj_update_snap(demo_selection_panel, LV_ANIM_ON);


    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
