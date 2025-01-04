#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "lvgl.h"
#include "board.h"
#include "esp_timer.h"
#include "button.h"
#include "mt8901.h"

#define TAG "MAIN"
#define ECO_O(y) (y > 0) ? -1 : 1
#define ECO_STEP(x) x ? ECO_O(x) : 0

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

extern void screen_init(void);

static lv_style_t style_btn;
static lv_style_t style_btn_pressed;
static lv_style_t style_btn_red;

static button_t *g_btn;
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
static lv_group_t *lv_group;
static lv_obj_t *lv_btn_1;
static lv_obj_t *lv_btn_2;

//static button_t *g_btn;

static lv_color_t darken(const lv_color_filter_dsc_t * dsc, lv_color_t color, lv_opa_t opa)
{
    LV_UNUSED(dsc);
    return lv_color_darken(color, opa);
}

/*void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  static int16_t cont_last = 0;
  int16_t cont_now = mt8901_get_count();
  data->enc_diff = ECO_STEP(cont_now - cont_last);
  cont_last = cont_now;
  if (button_isPressed(g_btn)) {
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}*/

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

void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  static int16_t cont_last = 0;
  int16_t cont_now = mt8901_get_count();
  data->enc_diff = ECO_STEP(cont_now - cont_last);
  cont_last = cont_now;
  if (button_isPressed(g_btn)) {
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

static void btn_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
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
  }
}

void init_lv_group() {
  lv_group = lv_group_create();
  lv_group_set_default(lv_group);

  lv_indev_t *cur_drv = NULL;
  for (;;) {
    cur_drv = lv_indev_get_next(cur_drv);
    if (!cur_drv) {
      break;
    }

    if (cur_drv->driver->type == LV_INDEV_TYPE_ENCODER) {
      lv_indev_set_group(cur_drv, lv_group);
    }
  }
}

/**
 * Create styles from scratch for buttons.
 */
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

void lvgl_task(void* arg) {
    screen_init();

    // Hardware Button
    g_btn = button_attch(3, 0, 10);

    // Magnetic Encoder
    mt8901_init(5, 6);

    // Tick interface for LVGL
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = increase_lvgl_tick,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    esp_timer_create(&periodic_timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, portTICK_PERIOD_MS * 1000);

    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = encoder_read;
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    lv_indev_drv_register(&indev_drv);

    //init_lv_group();
    lv_example_get_started_1();

    for (;;) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void) {
    xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);
}