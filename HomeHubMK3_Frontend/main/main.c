#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "lvgl.h"
#include "board.h"
#include "esp_timer.h"
#include "button.h"
#include "mt8901.h"
#include "bitmaps.h"
//#include "weather_images.h"

#define TAG "MAIN"
#define ECO_O(y) (y > 0) ? -1 : 1
#define ECO_STEP(x) x ? ECO_O(x) : 0

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

extern void screen_init(void);

static lv_style_t style_btn;
static lv_style_t style_btn_pressed;
static lv_style_t style_btn_red;

static button_t *g_btn;
static lv_group_t *lv_group;
static lv_obj_t *lv_btn_1;
static lv_obj_t *lv_btn_2;

lv_timer_t * timer;

const uart_port_t uart_num = UART_NUM_2;
QueueHandle_t uart_queue;


//static button_t *g_btn;

static lv_color_t darken(const lv_color_filter_dsc_t * dsc, lv_color_t color, lv_opa_t opa)
{
    LV_UNUSED(dsc);
    return lv_color_darken(color, opa);
}

static void style_init(void)
{
    /*Create a simple button style*/
    lv_style_init(&style_btn);
    lv_style_set_radius(&style_btn, 10);
    lv_style_set_bg_opa(&style_btn, LV_OPA_COVER);
    lv_style_set_bg_color(&style_btn, lv_palette_lighten(LV_PALETTE_GREY, 3));
    lv_style_set_bg_grad_color(&style_btn, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_bg_grad_dir(&style_btn, LV_GRAD_DIR_VER);

    lv_style_set_border_color(&style_btn, lv_color_black());
    lv_style_set_border_opa(&style_btn, LV_OPA_20);
    lv_style_set_border_width(&style_btn, 2);

    lv_style_set_text_color(&style_btn, lv_color_black());

    /*Create a style for the pressed state.
     *Use a color filter to simply modify all colors in this state*/
    static lv_color_filter_dsc_t color_filter;
    lv_color_filter_dsc_init(&color_filter, darken);
    lv_style_init(&style_btn_pressed);
    lv_style_set_color_filter_dsc(&style_btn_pressed, &color_filter);
    lv_style_set_color_filter_opa(&style_btn_pressed, LV_OPA_20);

    /*Create a red style. Change only some colors.*/
    lv_style_init(&style_btn_red);
    lv_style_set_bg_color(&style_btn_red, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&style_btn_red, lv_palette_lighten(LV_PALETTE_RED, 3));

}

void __qmsd_encoder_read(lv_indev_t *drv, lv_indev_data_t *data)
{
    static int16_t cont_last = 0;
    int16_t cont_now = mt8901_get_count();
    data->enc_diff = ECO_STEP(cont_now - cont_last);
    if(cont_now != cont_last){
        printf("Encoder Changed!\n");
        char* test_str = "Encoder Changed.\n";
        uart_write_bytes(UART_NUM_2, (const char*)test_str, strlen(test_str));
    }
    cont_last = cont_now;
    if (button_isPressed(g_btn)){
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

void __qsmd_encoder_init(void)
{
    static lv_indev_t *indev;

    g_btn = button_attch(3, 0, 10);
    mt8901_init(5,6);

	indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
	lv_indev_set_read_cb(indev, __qmsd_encoder_read);

  lv_group = lv_group_create();
  lv_group_set_default(lv_group);

  lv_indev_set_group(indev, lv_group);

}

static void btn_event_cb(lv_event_t *e) {
  /*lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *btn = lv_event_get_target(e);
  if (code == LV_EVENT_CLICKED) {

    uint8_t cnt = 0;
    if (btn == lv_btn_1) {
      static uint8_t cnt_1 = 0;
      cnt_1++;
      cnt = cnt_1;
    } else {
      static uint8_t cnt_2 = 0;
      cnt_2++;
      cnt = cnt_2;
    }
    //Get the first child of the button which is the label and change its text
    lv_obj_t *label = lv_obj_get_child(btn, 0);
    lv_label_set_text_fmt(label, "Button: %d", cnt);
  }*/
}

/**
 * Create styles from scratch for buttons.
 */
static void set_temp(void * bar, int32_t temp)
{
    lv_bar_set_value(bar, temp, LV_ANIM_ON);
}

void lv_example_get_started_1(void) {

  lv_btn_1 = lv_btn_create(lv_scr_act());                          /*Add a button the current screen*/
  lv_obj_set_size(lv_btn_1, 120, 50);                              /*Set its size*/
  lv_obj_align(lv_btn_1, LV_ALIGN_CENTER, 0, -40);                 /*Set its position*/
  lv_obj_add_event_cb(lv_btn_1, btn_event_cb, LV_EVENT_ALL, NULL); /*Assign a callback to the button*/

  lv_obj_t *label = lv_label_create(lv_btn_1); /*Add a label to the button*/
  lv_label_set_text(label, "Button");          /*Set the labels text*/
  lv_obj_center(label);

  lv_btn_2 = lv_btn_create(lv_scr_act());                                             /*Add a button the current screen*/
  lv_obj_set_style_bg_color(lv_btn_2, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN); /*Set its main color*/

  lv_obj_set_size(lv_btn_2, 120, 50);                              /*Set its size*/
  lv_obj_align(lv_btn_2, LV_ALIGN_CENTER, 0, 40);                  /*Set its position*/
  lv_obj_add_event_cb(lv_btn_2, btn_event_cb, LV_EVENT_ALL, NULL); /*Assign a callback to the button*/

  lv_obj_t *label2 = lv_label_create(lv_btn_2); /*Add a label to the button*/
  lv_label_set_text(label2, "Button");          /*Set the labels text*/
  lv_obj_center(label2);

    
}

void draw_UI_Main(){
    static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
    static lv_style_t style_knob;
    static lv_style_transition_dsc_t transition_dsc;
    lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 500, 0, NULL);

    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_10);
    lv_style_set_bg_color(&style_knob, lv_color_hex(0xFFFFFF));
    lv_style_set_border_color(&style_knob, lv_color_hex(0xFFFFFF));
    lv_style_set_border_width(&style_knob, 5);
    lv_style_set_radius(&style_knob, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_knob, 6); /*Makes the knob larger*/
    lv_style_set_transition(&style_knob, &transition_dsc);

    static lv_style_t style_indicator;

    static lv_style_t style_bg_temperature;
    static lv_color_t temperature_warm = { .red = 0xFF, .green = 0x96, .blue = 0x3C };  //2400K
    static lv_color_t temperature_cool = { .red = 0xF2, .green = 0xF2, .blue = 0xFF };  //7000K

    static lv_style_t style_bg_brightness;
    static lv_color_t brightness_dark = { .red = 0x1C, .green = 0x1C, .blue = 0x1C };  //dark
    static lv_color_t brightness_light = { .red = 0xFF, .green = 0xFF, .blue = 0xFF };  //7000K

    lv_style_init(&style_indicator);
    lv_style_set_border_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_color(&style_indicator, lv_color_hex(0x000000));

    lv_style_init(&style_bg_temperature);
    lv_style_set_bg_opa(&style_bg_temperature, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_temperature, temperature_warm);
    lv_style_set_bg_grad_color(&style_bg_temperature, temperature_cool);
    lv_style_set_bg_grad_dir(&style_bg_temperature, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_temperature, lv_color_hex(0xFFFFFF));
    

    lv_obj_t * bar = lv_slider_create(lv_screen_active());
    lv_obj_set_size(bar, 200, 20);
    lv_obj_align(bar, LV_ALIGN_CENTER, 0, 100);                  /*Set its position*/
    lv_bar_set_range(bar, 24, 70);
    lv_obj_add_style(bar, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(bar, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(bar, &style_bg_temperature, LV_PART_MAIN);
    
    
    lv_style_init(&style_bg_brightness);
    lv_style_set_bg_opa(&style_bg_brightness, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_brightness, brightness_dark);
    lv_style_set_bg_grad_color(&style_bg_brightness, brightness_light);
    lv_style_set_bg_grad_dir(&style_bg_brightness, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_brightness, lv_color_hex(0xFFFFFF));
    

    lv_obj_t * bar2 = lv_slider_create(lv_screen_active());
    
    lv_obj_set_size(bar2, 200, 20);
    lv_obj_align(bar2, LV_ALIGN_CENTER, 0, 150);                  /*Set its position*/
    lv_bar_set_range(bar2, 24, 70);
    lv_obj_add_style(bar2, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(bar2, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(bar2, &style_bg_brightness, LV_PART_MAIN);


    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);


    lv_obj_t * roller1 = lv_roller_create(lv_screen_active());
    lv_roller_set_options(roller1,
                          "January\n"
                          "February\n"
                          "March\n"
                          "April\n"
                          "May\n"
                          "June\n"
                          "July\n"
                          "August\n"
                          "September\n"
                          "October\n"
                          "November\n"
                          "December",
                          LV_ROLLER_MODE_INFINITE);

    lv_roller_set_visible_row_count(roller1, 4);
    lv_obj_center(roller1);
    lv_obj_add_event_cb(roller1, NULL, LV_EVENT_ALL, NULL);
    lv_obj_align(roller1, LV_ALIGN_CENTER, 150, 0);

    LV_IMAGE_DECLARE(WatchAway);
    LV_IMAGE_DECLARE(WatchPresent);

    static lv_obj_t * phoneImage;
    phoneImage = lv_image_create(lv_screen_active());
    lv_obj_align(phoneImage, LV_ALIGN_CENTER, -125, 0);
    lv_image_set_src(phoneImage, &WatchAway);

    static lv_obj_t * phoneImage2;
    phoneImage2 = lv_image_create(lv_screen_active());
    lv_obj_align(phoneImage2, LV_ALIGN_CENTER, -25, 0);
    lv_image_set_src(phoneImage2, &WatchPresent);

    static lv_obj_t * text_label_date;
    text_label_date = lv_label_create(lv_screen_active());
    lv_label_set_text(text_label_date, "1/17/2025");
    lv_obj_align(text_label_date, LV_ALIGN_CENTER, 0, -150);
    lv_obj_set_style_text_font((lv_obj_t*) text_label_date, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color((lv_obj_t*) text_label_date, lv_palette_main(LV_PALETTE_TEAL), 0);

    //lv_anim_t a;
    //lv_anim_init(&a);
    //lv_anim_set_exec_cb(&a, set_temp);
    //lv_anim_set_duration(&a, 3000);
    //lv_anim_set_playback_duration(&a, 3000);
    //lv_anim_set_var(&a, bar);
    //lv_anim_set_values(&a, -20, 40);
    //lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    //lv_anim_start(&a);
}

void lvgl_task(void* arg) {
    screen_init();

    __qsmd_encoder_init();
                                        
    // Tick interface for LVGL
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000);

    draw_UI_Main();

    for (;;) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO1, RX: IO2)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 1, 2, -1, -1));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 10, &uart_queue, 0));

    //ESP_ERROR_CHECK(uart_intr_config(uart_num, &uart_intr));

    // Enable UART RX FIFO full threshold and timeout interrupts
    //ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
    
    //Create a task to handler UART event from ISR
    //xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    //static uint32_t user_data = 10;
    //timer = lv_timer_create(uart_print, 500, &user_data);

    xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);

    
}