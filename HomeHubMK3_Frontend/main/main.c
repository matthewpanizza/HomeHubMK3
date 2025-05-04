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
#include "math.h"
#include "driver/ledc.h"
//#include "weather_images.h"

#define TAG "MAIN"
#define ECO_O(y) (y > 0) ? -1 : 1
#define ECO_STEP(x) x ? ECO_O(x) : 0

#define BUF_SIZE (1024)
#define RX_BUF_SIZE (BUF_SIZE)
#define MAX_RX_COMMANDS (10)

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

extern void screen_init(void);
static void processCommand(const char* input, uint16_t length);

LV_IMAGE_DECLARE(WatchAway);
LV_IMAGE_DECLARE(WatchAway);
LV_IMAGE_DECLARE(WatchPresent);
LV_IMAGE_DECLARE(iPhoneAway);
LV_IMAGE_DECLARE(bulbicon);
LV_IMAGE_DECLARE(bulbfill);

// Hardware
static button_t *g_btn;
const uart_port_t uart_num = UART_NUM_2;
QueueHandle_t uart_queue;

//UI Groups
static lv_group_t *lv_group;

// UI Elements
static lv_obj_t * brightnessSlider;
static lv_obj_t * colorSlider;
static lv_obj_t * text_label_date;
static lv_obj_t * phoneImage;
static lv_obj_t * watchImage;
static lv_obj_t * bulbOutline;
static lv_obj_t * bulbFill;

//Colors

static lv_color_t color_warm = { .red = 0xFF, .green = 0x96, .blue = 0x3C };  //2400K
static lv_color_t color_cool = { .red = 0xF2, .green = 0xF2, .blue = 0xFF };  //7000K
lv_color_t bulbFillColor = { .red = 0xFF, .green = 0x96, .blue = 0x3C };  //2400K
lv_color_t bulbOutlineColor = { .red = 0xFF, .green = 0xFF, .blue = 0xFF };  //White

// Styling
static lv_style_t styleBulbOutline;
static lv_style_t styleBulbFill;

//Global variables for UI Operation
uint8_t timeHours = 0;
uint8_t timeMinutes = 0;
bool watchPresent = false;
bool phonePresent = false;
uint8_t backlightPercentage = 10;
uint8_t lastBacklightPercentage = 0;

static lv_color_t darken(const lv_color_filter_dsc_t * dsc, lv_color_t color, lv_opa_t opa)
{
    LV_UNUSED(dsc);
    return lv_color_darken(color, opa);
}

void __qmsd_encoder_read(lv_indev_t *drv, lv_indev_data_t *data)
{
    static int16_t cont_last = 0;
    int16_t cont_now = mt8901_get_count();
    data->enc_diff = ECO_STEP(cont_now - cont_last);
    if(cont_now != cont_last){
        printf("Encoder Changed!\n");
        //char* test_str = "Encoder Changed.\n";
        //uart_write_bytes(UART_NUM_2, (const char*)test_str, strlen(test_str));
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

void configure_backlight_PWM(uint32_t frequency, uint8_t duty_cycle_percent) {
    // Configure the LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,  // Low-speed mode
        .timer_num        = LEDC_TIMER_0,         // Timer 0
        .duty_resolution  = LEDC_TIMER_8_BIT,     // 8-bit resolution
        .freq_hz          = frequency,            // Frequency in Hz
        .clk_cfg          = LEDC_AUTO_CLK         // Auto select clock source
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure the LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = 38,                     // GPIO pin 38
        .speed_mode     = LEDC_LOW_SPEED_MODE,    // Low-speed mode
        .channel        = LEDC_CHANNEL_0,         // Channel 0
        .timer_sel      = LEDC_TIMER_0,           // Use Timer 0
        .duty           = 0,                      // Initial duty cycle (0%)
        .hpoint         = 0                       // High point
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Set the initial duty cycle
    uint32_t duty = (duty_cycle_percent * ((1 << LEDC_TIMER_8_BIT) - 1)) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void update_backlight_PWM(uint8_t duty_cycle_percent) {
    // Update the duty cycle
    uint32_t duty = (duty_cycle_percent * ((1 << LEDC_TIMER_8_BIT) - 1)) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

// Define the queue handle
static QueueHandle_t commandQueue;

// Update the rx_task function
static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    static char buffer[RX_BUF_SIZE + 1];
    static int buffer_len = 0;

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            for (int i = 0; i < rxBytes; i++) {
                if (data[i] == '\n') {
                    buffer[buffer_len] = '\0'; // Null-terminate the string

                    // Push the command into the queue
                    char* command = strdup(buffer); // Allocate memory for the command
                    if (command != NULL) {
                        if (xQueueSend(commandQueue, &command, portMAX_DELAY) != pdPASS) {
                            free(command); // Free memory if the queue is full
                            ESP_LOGE("RX TASK", "Failed to enqueue command");
                        }
                    }

                    buffer_len = 0; // Reset the buffer length for the next command
                } else {
                    if (buffer_len < RX_BUF_SIZE) {
                        buffer[buffer_len++] = data[i]; // Append character to buffer
                    }
                }
            }
        }
    }
    free(data);
}

static void check_command_queue(){
    char* command;
    if (xQueueReceive(commandQueue, &command, 0) == pdPASS) {
        processCommand(command, strlen(command));
        free(command); // Free the allocated memory after processing
    }
}

static void brightness_changed_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    uint32_t sliderValue = lv_slider_get_value(slider);

    char output[10];
    lv_snprintf(output, sizeof(output), "UB%d\n", sliderValue);
    uart_write_bytes(UART_NUM_2, (const char*)output, strlen(output));
}

static void color_changed_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    uint32_t sliderValue = lv_slider_get_value(slider);

    char output[10];
    lv_snprintf(output, sizeof(output), "UC%d\n", sliderValue);
    uart_write_bytes(UART_NUM_2, (const char*)output, strlen(output));
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

// Function to animate the slider value
void animate_slider(lv_obj_t *slider, int32_t new_value) {
    lv_anim_t a;
    lv_anim_init(&a); // Initialize the animation structure
    lv_anim_set_var(&a, slider); // Set the target object
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_slider_set_value); // Set the function to apply the animation
    lv_anim_set_values(&a, lv_slider_get_value(slider), new_value); // Set the start and end values
    lv_anim_set_time(&a, 1000); // Set the duration of the animation (in milliseconds)
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out); // Set the easing function for smooth transitions
    lv_anim_start(&a); // Start the animation
}


static void processCommand(const char* input, uint16_t length) {
    //First character is the node identifier (U = UI, M = Main Controller, S = Secondary Controller)
    //Second character is the command
    //Remaining characters are the payload. Process the data based on the type of command and where it's coming from

    if(length < 0) return; //Invalid length

    static const char *CMD_TASK_TAG = "CMD_TASK";
    esp_log_level_set(CMD_TASK_TAG, ESP_LOG_INFO);
    ESP_LOGI(CMD_TASK_TAG, "Read bytes: %s", input);

    char payload[length];
    for(int i = 2; i < length; i++){
        payload[i-2] = input[i];
    }
    //ESP_LOGI(CMD_TASK_TAG, "Read payload: %s", payload);

    uint8_t targetBrightness = 0;
    uint8_t targetColor = 0;
    int fillR, fillG, fillB = 0;
    int tHours, tMinutes = 0;
    if(input[0] == 'M'){
        switch(input[1]){
            case 'S': //Device status update - Shows if one of the BLE devices is discovered
                break;
            case 'T': //Time change - gets the time from the main controller and sets it to the UI
                //Format is "HH:MM"
                if (sscanf(payload, "%2d %2d", &tHours, &tMinutes) == 2) {
                    //ESP_LOGI(CMD_TASK_TAG, "Got Time Command - %u:%u", timeHours, timeMinutes);
                    timeHours = tHours;
                    timeMinutes = tMinutes;
                } else {
                    //ESP_LOGI(CMD_TASK_TAG, "Invalid time format");
                }
        
                
                break;
            case 'F':    //Bulb fill command. Takes RGB value and sets bulb fill color
                //Format is "RRGGBB"
                if (sscanf(payload, "%x %x %x", &fillR, &fillG, &fillB) == 3) {
                    //ESP_LOGI(CMD_TASK_TAG, "Got Bulb Fill Command - %02X:%02X:%02X", fillR, fillG, fillB);
                    bulbFillColor.red = fillR;
                    bulbFillColor.green = fillG;
                    bulbFillColor.blue = fillB;
                    lv_style_set_img_recolor(&styleBulbFill, bulbFillColor);             // Set the recolor
                    lv_obj_report_style_change(&styleBulbFill);

                } else {
                    //ESP_LOGI(CMD_TASK_TAG, "Invalid RGB format");
                }
                break;
            case 'O':    //Bulb outline command. Takes RGB value and sets bulb fill color
                //Format is "RRGGBB"
                if (sscanf(payload, "%x %x %x", &fillR, &fillG, &fillB) == 3) {
                    //ESP_LOGI(CMD_TASK_TAG, "Got Bulb Fill Command - %02X:%02X:%02X", fillR, fillG, fillB);
                    bulbOutlineColor.red = fillR;
                    bulbOutlineColor.green = fillG;
                    bulbOutlineColor.blue = fillB;
                    lv_style_set_img_recolor(&styleBulbOutline, bulbOutlineColor);             // Set the recolor
                    lv_obj_report_style_change(&styleBulbOutline);

                } else {
                    //ESP_LOGI(CMD_TASK_TAG, "Invalid RGB format");
                }
                break;
            case 'B': //Brightness change - gets the brightness from the main controller and sets it to the UI
                targetBrightness = atoi(payload);
                if(targetBrightness > 100) targetBrightness = 100;
                animate_slider(brightnessSlider, targetBrightness);
                break;
            case 'C': //Color temperature change - gets the color from the main controller and sets it to the UI
                targetColor = atoi(payload);
                if(targetColor < 24) targetColor = 24;
                animate_slider(colorSlider, targetColor);
                break;
            case 'N': //Network status - information about the network status of the main controller
                break;
            case 'L':
                //Backlight change - gets the backlight from the main controller and sets it to the UI
                backlightPercentage = atoi(payload);
                if(backlightPercentage > 100) backlightPercentage = 100;
                break;
            case 'W': //Watch status - information about the watch status (present or away)
                if(payload[0] == '1'){
                    watchPresent = true;
                    //ESP_LOGI(CMD_TASK_TAG, "Watch Present");
                } else if(payload[0] == '0'){
                    watchPresent = false;
                    //ESP_LOGI(CMD_TASK_TAG, "Watch Away");
                } else {
                    //ESP_LOGI(CMD_TASK_TAG, "Invalid Watch Status Payload: %s", payload);
                }
                break;
            case 'P': //Phone status - information about the phone status (present or away)
                if(payload[0] == '1'){
                    phonePresent = true;
                    //ESP_LOGI(CMD_TASK_TAG, "Phone Present");
                } else if(payload[0] == '0'){
                    phonePresent = false;
                    //ESP_LOGI(CMD_TASK_TAG, "Phone Away");
                } else {
                    //ESP_LOGI(CMD_TASK_TAG, "Invalid Phone Status Payload: %s", payload);
                }
                break;
            default:
                break;
        }
    }
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

    static lv_style_t style_bg_brightness;
    static lv_color_t brightness_dark = { .red = 0x1C, .green = 0x1C, .blue = 0x1C };  //dark
    static lv_color_t brightness_light = { .red = 0xFF, .green = 0xFF, .blue = 0xFF };  //7000K

    lv_style_init(&style_indicator);
    lv_style_set_border_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_color(&style_indicator, lv_color_hex(0x000000));

    lv_style_init(&style_bg_temperature);
    lv_style_set_bg_opa(&style_bg_temperature, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_temperature, color_warm);
    lv_style_set_bg_grad_color(&style_bg_temperature, color_cool);
    lv_style_set_bg_grad_dir(&style_bg_temperature, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_temperature, lv_color_hex(0xFFFFFF));
    

    colorSlider = lv_slider_create(lv_screen_active());

    lv_obj_add_event_cb(colorSlider, color_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_set_size(colorSlider, 200, 20);
    lv_obj_align(colorSlider, LV_ALIGN_CENTER, 0, 100);                  /*Set its position*/
    lv_bar_set_range(colorSlider, 24, 70);
    lv_obj_add_style(colorSlider, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(colorSlider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(colorSlider, &style_bg_temperature, LV_PART_MAIN);
    
    
    lv_style_init(&style_bg_brightness);
    lv_style_set_bg_opa(&style_bg_brightness, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_brightness, brightness_dark);
    lv_style_set_bg_grad_color(&style_bg_brightness, brightness_light);
    lv_style_set_bg_grad_dir(&style_bg_brightness, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_brightness, lv_color_hex(0xFFFFFF));
    

    brightnessSlider = lv_slider_create(lv_screen_active());

    lv_obj_add_event_cb(brightnessSlider, brightness_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    lv_obj_set_size(brightnessSlider, 200, 20);
    lv_obj_align(brightnessSlider, LV_ALIGN_CENTER, 0, 150);                  /*Set its position*/
    lv_bar_set_range(brightnessSlider, 0, 100);
    lv_obj_add_style(brightnessSlider, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(brightnessSlider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(brightnessSlider, &style_bg_brightness, LV_PART_MAIN);


    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);


    /* lv_obj_t * roller1 = lv_roller_create(lv_screen_active());
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
    lv_obj_align(roller1, LV_ALIGN_CENTER, 150, 0);*/

    lv_style_init(&styleBulbOutline);
    lv_style_set_img_recolor(&styleBulbOutline, lv_color_hex(0xFFFFFF));             // Set the recolor
    lv_style_set_img_recolor_opa(&styleBulbOutline, LV_OPA_COVER);  // Set opacity to full recolor

    lv_style_init(&styleBulbFill);
    lv_style_set_img_recolor(&styleBulbFill, color_warm);             // Set the recolor
    lv_style_set_img_recolor_opa(&styleBulbFill, LV_OPA_COVER);  // Set opacity to full recolor

    
    watchImage = lv_image_create(lv_screen_active());
    lv_obj_align(watchImage, LV_ALIGN_CENTER, -125, 0);
    lv_image_set_src(watchImage, &WatchAway);

    phoneImage = lv_image_create(lv_screen_active());
    lv_obj_align(phoneImage, LV_ALIGN_CENTER, 125, 0);
    lv_image_set_src(phoneImage, &iPhoneAway);


    bulbFill = lv_image_create(lv_screen_active());
    lv_obj_add_style(bulbFill, &styleBulbFill, LV_PART_MAIN);
    lv_obj_align(bulbFill, LV_ALIGN_CENTER, 0, 0);
    lv_image_set_src(bulbFill, &bulbfill);


    bulbOutline = lv_image_create(lv_screen_active());
    lv_obj_add_style(bulbOutline, &styleBulbOutline, LV_PART_MAIN);
    lv_obj_align(bulbOutline, LV_ALIGN_CENTER, 0, 0);
    lv_image_set_src(bulbOutline, &bulbicon);

    
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

void update_UI_Main(){
    char timeLabel[10];
    lv_snprintf(timeLabel, sizeof(timeLabel), "%2u:%02u", timeHours, timeMinutes);
    lv_label_set_text(text_label_date, timeLabel);

    static bool lastWatchStatus = false;
    static bool lastPhoneStatus = false;
    if(watchPresent != lastWatchStatus){
        lastWatchStatus = watchPresent;
        if(watchPresent){
            lv_image_set_src(watchImage, &WatchPresent);
        } else {
            lv_image_set_src(watchImage, &WatchAway);
        }
    }
    if(phonePresent != lastPhoneStatus){
        lastPhoneStatus = phonePresent;
        if(phonePresent){
            lv_image_set_src(phoneImage, &iPhoneIconPresent);
        } else {
            lv_image_set_src(phoneImage, &iPhoneAway);
        }
    }

    if(backlightPercentage != lastBacklightPercentage){
        lastBacklightPercentage = backlightPercentage;
        update_backlight_PWM(backlightPercentage);
    }
}

void lvgl_task(void* arg) {
    //configure_backlight_PWM(1000, 0); // Set the backlight to 100% brightness

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
        update_UI_Main();
        lv_task_handler();
        check_command_queue();
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

    // Create the command queue
    commandQueue = xQueueCreate(10, sizeof(char*));

    xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 1024*2, NULL, 4, NULL, 0);

    configure_backlight_PWM(1000, 10); // Set frequency to 1 kHz and duty cycle to 50%
    //xTaskCreatePinnedToCore(command_processor_task, "cmd_prc_task", 1024*2, NULL, 4, NULL, 0);

    
}