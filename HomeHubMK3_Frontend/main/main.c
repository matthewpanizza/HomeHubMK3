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

// UART Parameters
#define BUF_SIZE (1024)
#define RX_BUF_SIZE (BUF_SIZE)
#define MAX_RX_COMMANDS (10)

static void increase_lvgl_tick(void* arg) {
    lv_tick_inc(portTICK_PERIOD_MS);
}

extern void screen_init(void);
static void processCommand(const char* input, uint16_t length);

// Image declarations for LVGL graphics
LV_IMAGE_DECLARE(WatchAway);
LV_IMAGE_DECLARE(WatchAway);
LV_IMAGE_DECLARE(WatchPresent);
LV_IMAGE_DECLARE(iPhoneAway);
LV_IMAGE_DECLARE(bulbicon);
LV_IMAGE_DECLARE(bulbfill);

// Hardware globals
static button_t *g_btn;
const uart_port_t uart_num = UART_NUM_2;
QueueHandle_t uart_queue;
static QueueHandle_t commandQueue;

// UI Groups
static lv_group_t *lv_group;            // Main UI item group

// UI Elements
static lv_obj_t * brightnessSlider;     // Slider to control smart bulb brightness
static lv_obj_t * colorSlider;          // Slider to control smart bulb color temperature
static lv_obj_t * text_label_date;      // Label to display the date
static lv_obj_t * phoneImage;           // Image to show if a phone is detected from BLE
static lv_obj_t * watchImage;           // Image to show if a smart watch is detected from BLE
static lv_obj_t * bulbOutline;          // Image for bulb glyph outline bitmap
static lv_obj_t * bulbFill;             // Image for bulb glyph fill bitmap
static lv_obj_t * toggleButton;         // Button to toggle the lights on/off
static lv_obj_t * toggleButtonLabel;    // Label for the toggle button

// Colors
static lv_color_t color_warm = { .red = 0xFF, .green = 0x96, .blue = 0x3C };        //2400K
static lv_color_t color_cool = { .red = 0xF2, .green = 0xF2, .blue = 0xFF };        //7000K
static lv_color_t brightness_dark = { .red = 0x1C, .green = 0x1C, .blue = 0x1C };   //dark
static lv_color_t brightness_light = { .red = 0xFF, .green = 0xFF, .blue = 0xFF };  //7000K
lv_color_t bulbFillColor = { .red = 0xFF, .green = 0x96, .blue = 0x3C };            //2400K
lv_color_t bulbOutlineColor = { .red = 0xFF, .green = 0xFF, .blue = 0xFF };         //White

// Styling
static lv_style_t styleBulbOutline;     // Style for the bitmap of the bulb glyph outline
static lv_style_t styleBulbFill;        // Style for the bitmap of the bulb glyph fill
static lv_style_t styleToggleButton;    // Style for the toggle lights button
static lv_style_t style_indicator;      // Style for the inner part of the slider knob
static lv_style_t style_bg_temperature; // Style for the background of the color temperature slider
static lv_style_t style_bg_brightness;  // Style for the background of the brightness slider

// Global variables for UI Operation
uint8_t timeHours = 0;                  // Clock hours
uint8_t timeMinutes = 0;                // Clock minutes
bool watchPresent = false;              // Watch presence status
bool phonePresent = false;              // Phone presence status
uint8_t backlightPercentage = 10;       // LCD backlight percentage (0-100)
uint8_t lastBacklightPercentage = 0;    // Percentage of LCD backlight last time it was updated (0-100)

static lv_color_t darken(const lv_color_filter_dsc_t * dsc, lv_color_t color, lv_opa_t opa)
{
    LV_UNUSED(dsc);
    return lv_color_darken(color, opa);
}


/// @brief Callback function to read the encoder and button state
/// @param drv input device driver object
/// @param data data from the input device
void __qmsd_encoder_read(lv_indev_t *drv, lv_indev_data_t *data)
{
    static int16_t cont_last = 0;
    int16_t cont_now = mt8901_get_count();
    data->enc_diff = ECO_STEP(cont_now - cont_last);
    if(cont_now != cont_last){
        printf("Encoder Changed!\n");
    }
    cont_last = cont_now;
    if (button_isPressed(g_btn)){
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}


/// @brief Function to initialize the encoder and button and register them with LVGL as input devices
void __qsmd_encoder_init(void)
{
    // Create an input device object for an encoder
    static lv_indev_t *indev;

    // Set up a button on pin 3 with a Switch-to-Ground configuration and a 10ms debounce time
    g_btn = button_attch(3, 0, 10);

    // Initialize the MT8901 encoder on pins 5 and 6
    mt8901_init(5,6);

    // Set up the LVGL input device for the encoder and register the callback function for when the button/encoder changes
	indev = lv_indev_create();
	lv_indev_set_type(indev, LV_INDEV_TYPE_ENCODER);
	lv_indev_set_read_cb(indev, __qmsd_encoder_read);

    // Create a default group for the UI elements that the encoder will control
    lv_group = lv_group_create();
    lv_group_set_default(lv_group);

    // Assign the input device control to the default main group
    lv_indev_set_group(indev, lv_group);

}


/// @brief Function to configure the backlight PWM for the LCD. Uses the LEDC driver to control the brightness of the backlight.
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
        .gpio_num       = 38,                     // LCD backlight on GPIO pin 38
        .speed_mode     = LEDC_LOW_SPEED_MODE,    // Low-speed mode (only one available on ESP32-S3)
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


/// @brief Updates the PWM duty cycle for the backlight based on the given percentage.
/// @param duty_cycle_percent backlight percentage (0-100)
void update_backlight_PWM(uint8_t duty_cycle_percent) {
    // Update the duty cycle
    uint32_t duty = (duty_cycle_percent * ((1 << LEDC_TIMER_8_BIT) - 1)) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}


/// @brief Function to pop messages from the command queue and process them. Should be run from the same thread as LVGL to ensure proper UI behavior.
/// @note This function is called from the main loop of LVGL and should be called periodically to process commands.
static void check_command_queue(){
    char* command;
    // Check if there are any commands in the queue
    if (xQueueReceive(commandQueue, &command, 0) == pdPASS) {
        processCommand(command, strlen(command));   // Call the command processing function with the command received
        free(command);                              // Free the allocated memory after processing
    }
}


/// @brief Callback function for when the brightness slider value changes
/// @param e event data
static void brightness_changed_cb(lv_event_t *e) {
    // Get the object that triggered the event and its value
    lv_obj_t *slider = lv_event_get_target(e);
    uint32_t sliderValue = lv_slider_get_value(slider);

    // Send the value over UART to the main controller
    char output[10];
    lv_snprintf(output, sizeof(output), "UB%d\n", sliderValue);
    uart_write_bytes(UART_NUM_2, (const char*)output, strlen(output));
}


/// @brief Callback function for when the color slider value changes
/// @param e event data
static void color_changed_cb(lv_event_t *e) {
    // Get the object that triggered the event and its value
    lv_obj_t *slider = lv_event_get_target(e);
    uint32_t sliderValue = lv_slider_get_value(slider);

    // Send the value over UART to the main controller
    char output[10];
    lv_snprintf(output, sizeof(output), "UC%d\n", sliderValue);
    uart_write_bytes(UART_NUM_2, (const char*)output, strlen(output));
}


/// @brief Callback function for when the toggle lights button is pressed
/// @param e event data
static void toggle_event_cb(lv_event_t *e) {
    // Send the string "UG" over UART when the button is pressed to indicate a toggle
    const char *message = "UG\n";
    uart_write_bytes(UART_NUM_2, message, strlen(message));
}


/// @brief Function to animate the slider value change with a smooth transition
/// @param slider slider object to animate
/// @param new_value new value to set the slider to
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


/// @brief Function to parse strings commands from the main controller and update the UI accordingly
/// @param input string command received from the main controller. Should be '\0' terminated.
/// @param length number of bytes in the command
static void processCommand(const char* input, uint16_t length) {
    //First character is the node identifier (U = UI, M = Main Controller, S = Secondary Controller)
    //Second character is the command
    //Remaining characters are the payload. Process the data based on the type of command and where it's coming from

    if(length < 0) return; //Invalid length

    static const char *CMD_TASK_TAG = "CMD_TASK";
    esp_log_level_set(CMD_TASK_TAG, ESP_LOG_INFO);
    ESP_LOGI(CMD_TASK_TAG, "Read bytes: %s", input);

    // Extract the payload from the input string
    char payload[length];
    for(int i = 2; i < length; i++){
        payload[i-2] = input[i];
    }
    ESP_LOGI(CMD_TASK_TAG, "Read payload: %s", payload);

    // Initialize variables for parsing the command
    uint8_t targetBrightness = 0;
    uint8_t targetColor = 0;
    int fillR, fillG, fillB = 0;
    int tHours, tMinutes = 0;

    //Check if command is from the main controller
    if(input[0] == 'M'){
        switch(input[1]){
            case 'S': //Device status update - Shows if one of the BLE devices is discovered
                //TODO
                break;
            case 'T': //Time change - gets the time from the main controller and sets it to the UI
                //Format is "HH:MM"
                if (sscanf(payload, "%2d %2d", &tHours, &tMinutes) == 2) {
                    ESP_LOGI(CMD_TASK_TAG, "Got Time Command - %u:%u", tHours, tMinutes);
                    timeHours = tHours;           // Copy parsed values to the global variables
                    timeMinutes = tMinutes;
                } else {
                    ESP_LOGI(CMD_TASK_TAG, "Invalid time format");
                }
                break;
            case 'F': //Bulb fill command. Takes RGB value and sets bulb fill color
                //Format is "RRGGBB"
                if (sscanf(payload, "%x %x %x", &fillR, &fillG, &fillB) == 3) {
                    ESP_LOGI(CMD_TASK_TAG, "Got Bulb Fill Command - %02X:%02X:%02X", fillR, fillG, fillB);
                    bulbFillColor.red = fillR;    // Copy parsed values to the global variables
                    bulbFillColor.green = fillG;
                    bulbFillColor.blue = fillB;
                    lv_style_set_img_recolor(&styleBulbFill, bulbFillColor);             // Set the recolor
                    lv_obj_report_style_change(&styleBulbFill);

                } else {
                    ESP_LOGI(CMD_TASK_TAG, "Invalid RGB format");
                }
                break;
            case 'O': //Bulb outline command. Takes RGB value and sets bulb fill color
                //Format is "RRGGBB"
                if (sscanf(payload, "%x %x %x", &fillR, &fillG, &fillB) == 3) {
                    ESP_LOGI(CMD_TASK_TAG, "Got Bulb Fill Command - %02X:%02X:%02X", fillR, fillG, fillB);
                    bulbOutlineColor.red = fillR;   // Copy parsed values to the global variables
                    bulbOutlineColor.green = fillG;
                    bulbOutlineColor.blue = fillB;
                    lv_style_set_img_recolor(&styleBulbOutline, bulbOutlineColor);             // Set the recolor
                    lv_obj_report_style_change(&styleBulbOutline);

                } else {
                    ESP_LOGI(CMD_TASK_TAG, "Invalid RGB format");
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
                //TODO
                break;
            case 'L': //Backlight change - gets the backlight from the main controller and sets it to the UI
                backlightPercentage = atoi(payload);
                if(backlightPercentage > 100) backlightPercentage = 100;
                break;
            case 'W': //Watch status - information about the watch status (present or away)
                if(payload[0] == '1'){
                    watchPresent = true;
                    ESP_LOGI(CMD_TASK_TAG, "Watch Present");
                } else if(payload[0] == '0'){
                    watchPresent = false;
                    ESP_LOGI(CMD_TASK_TAG, "Watch Away");
                } else {
                    ESP_LOGI(CMD_TASK_TAG, "Invalid Watch Status Payload: %s", payload);
                }
                break;
            case 'P': //Phone status - information about the phone status (present or away)
                if(payload[0] == '1'){
                    phonePresent = true;
                    ESP_LOGI(CMD_TASK_TAG, "Phone Present");
                } else if(payload[0] == '0'){
                    phonePresent = false;
                    ESP_LOGI(CMD_TASK_TAG, "Phone Away");
                } else {
                    ESP_LOGI(CMD_TASK_TAG, "Invalid Phone Status Payload: %s", payload);
                }
                break;
            default:
                break;
        }
    }
}


/// @brief Main function to draw the LVGL UI elements on the screen
void draw_UI_Main(){
    static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
    static lv_style_t style_knob;
    static lv_style_transition_dsc_t transition_dsc;
    lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 500, 0, NULL);

    // Set UI background color to black
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);

    // Style for the knob on the color and brightness sliders
    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_10);
    lv_style_set_bg_color(&style_knob, lv_color_hex(0xFFFFFF));
    lv_style_set_border_color(&style_knob, lv_color_hex(0xFFFFFF));
    lv_style_set_border_width(&style_knob, 5);
    lv_style_set_radius(&style_knob, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_knob, 6); /*Makes the knob larger*/
    lv_style_set_transition(&style_knob, &transition_dsc);

    // Set up style for the inside of the slider knobs
    lv_style_init(&style_indicator);
    lv_style_set_border_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_0);
    lv_style_set_bg_color(&style_indicator, lv_color_hex(0x000000));

    // Set up style for the background of the color temperature slider
    lv_style_init(&style_bg_temperature);
    lv_style_set_bg_opa(&style_bg_temperature, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_temperature, color_warm);
    lv_style_set_bg_grad_color(&style_bg_temperature, color_cool);
    lv_style_set_bg_grad_dir(&style_bg_temperature, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_temperature, lv_color_hex(0xFFFFFF));

    // Set up style for the background of the brightness slider
    lv_style_init(&style_bg_brightness);
    lv_style_set_bg_opa(&style_bg_brightness, LV_OPA_COVER);
    lv_style_set_bg_color(&style_bg_brightness, brightness_dark);
    lv_style_set_bg_grad_color(&style_bg_brightness, brightness_light);
    lv_style_set_bg_grad_dir(&style_bg_brightness, LV_GRAD_DIR_HOR);
    lv_style_set_border_color(&style_bg_brightness, lv_color_hex(0xFFFFFF));
    
    // Create the color temperature slider on the main screen
    colorSlider = lv_slider_create(lv_screen_active());

    // Set up the callback function for when the color temperature slider value changes
    lv_obj_add_event_cb(colorSlider, color_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Apply parameters to the color temperature slider
    lv_obj_set_size(colorSlider, 200, 20);
    lv_obj_align(colorSlider, LV_ALIGN_CENTER, 0, 100);                  /*Set its position*/
    lv_bar_set_range(colorSlider, 24, 70);
    lv_obj_add_style(colorSlider, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(colorSlider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(colorSlider, &style_bg_temperature, LV_PART_MAIN);

    // Create the brightness slider on the main screen
    brightnessSlider = lv_slider_create(lv_screen_active());

    // Set up the callback function for when the brightness slider value changes
    lv_obj_add_event_cb(brightnessSlider, brightness_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Apply parameters to the brightness slider
    lv_obj_set_size(brightnessSlider, 200, 20);
    lv_obj_align(brightnessSlider, LV_ALIGN_CENTER, 0, 150);                  /*Set its position*/
    lv_bar_set_range(brightnessSlider, 0, 100);
    lv_obj_add_style(brightnessSlider, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(brightnessSlider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(brightnessSlider, &style_bg_brightness, LV_PART_MAIN);

    // Create the style for the bulb outline
    lv_style_init(&styleBulbOutline);
    lv_style_set_img_recolor(&styleBulbOutline, lv_color_hex(0xFFFFFF));    // Set the recolor
    lv_style_set_img_recolor_opa(&styleBulbOutline, LV_OPA_COVER);          // Set opacity to full recolor

    // Create the style for the fill outline
    lv_style_init(&styleBulbFill);
    lv_style_set_img_recolor(&styleBulbFill, color_warm);                   // Set the recolor
    lv_style_set_img_recolor_opa(&styleBulbFill, LV_OPA_COVER);             // Set opacity to full recolor

    // Create image for apple watch icon
    watchImage = lv_image_create(lv_screen_active());
    lv_obj_align(watchImage, LV_ALIGN_CENTER, -125, 0);
    lv_image_set_src(watchImage, &WatchAway);

    // Create image for the iPhone icon
    phoneImage = lv_image_create(lv_screen_active());
    lv_obj_align(phoneImage, LV_ALIGN_CENTER, 125, 0);
    lv_image_set_src(phoneImage, &iPhoneAway);

    // Create image for smart bulb icon bitmap
    bulbFill = lv_image_create(lv_screen_active());
    lv_obj_add_style(bulbFill, &styleBulbFill, LV_PART_MAIN);
    lv_obj_align(bulbFill, LV_ALIGN_CENTER, 0, 0);
    lv_image_set_src(bulbFill, &bulbfill);

    // Create image for smart bulb icon bitmap
    bulbOutline = lv_image_create(lv_screen_active());
    lv_obj_add_style(bulbOutline, &styleBulbOutline, LV_PART_MAIN);
    lv_obj_align(bulbOutline, LV_ALIGN_CENTER, 0, 0);
    lv_image_set_src(bulbOutline, &bulbicon);

    // Create a label for the date
    text_label_date = lv_label_create(lv_screen_active());
    lv_label_set_text(text_label_date, "1/17/2025");
    lv_obj_align(text_label_date, LV_ALIGN_CENTER, 0, -150);
    lv_obj_set_style_text_font((lv_obj_t*) text_label_date, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color((lv_obj_t*) text_label_date, lv_palette_main(LV_PALETTE_TEAL), 0);

    // Create a style for the button
    lv_style_init(&styleToggleButton);
    lv_style_set_bg_color(&styleToggleButton, lv_color_hex(0x808080)); // Set background color to gray
    lv_style_set_bg_opa(&styleToggleButton, LV_OPA_COVER);             // Set full opacity
    lv_style_set_border_color(&styleToggleButton, lv_color_hex(0x000000)); // Set border color to black
    lv_style_set_border_width(&styleToggleButton, 2);                  // Set border width
    lv_style_set_radius(&styleToggleButton, 5);                        // Set corner radius

    // Create a button in the bottom center
    toggleButton = lv_btn_create(lv_screen_active());
    lv_obj_align(toggleButton, LV_ALIGN_BOTTOM_MID, 0, -20); // Align to the bottom center with a small offset
    lv_obj_set_size(toggleButton, 140, 40); // Set button size

    // Apply the style to the button
    lv_obj_add_style(toggleButton, &styleToggleButton, LV_PART_MAIN);

    // Add a label to the button
    toggleButtonLabel = lv_label_create(toggleButton);
    lv_label_set_text(toggleButtonLabel, "Toggle Lights");
    lv_obj_center(toggleButtonLabel); // Center the label on the button

    // Add an event callback to the button
    lv_obj_add_event_cb(toggleButton, toggle_event_cb, LV_EVENT_CLICKED, NULL);

}


/// @brief Function to update the UI elements based on the current state of the system
void update_UI_Main(){

    //Update the clock value
    char timeLabel[10];
    lv_snprintf(timeLabel, sizeof(timeLabel), "%2u:%02u", timeHours, timeMinutes);
    lv_label_set_text(text_label_date, timeLabel);

    // Update the watch and phone presence status
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

    // Update the backlight brightness if it has changed
    if(backlightPercentage != lastBacklightPercentage){
        lastBacklightPercentage = backlightPercentage;
        update_backlight_PWM(backlightPercentage);
    }
}


/// @brief Thread function called periodically to check for any commands received over UART
/// @param arg 
/// @note This function is called from a separate thread from LVGL and pushes to a thread-safe queue
static void rx_task(void *arg)
{
    // Allocate memory for the command queue
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    static char buffer[RX_BUF_SIZE + 1];
    static int buffer_len = 0;

    while (1) {
        // Check if data is available on the UART port
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1 / portTICK_PERIOD_MS);

        // If we have data, loop over bytes until a newline is found and push the command to the queue for processing
        if (rxBytes > 0) {
            for (int i = 0; i < rxBytes; i++) {
                // Check for newline character to identify end of command
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


/// @brief Main task function for LVGL. Initializes the screen and starts the LVGL task loop.
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
        update_UI_Main();
        lv_task_handler();
        check_command_queue();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/// @brief Main function. Initializes the hardware, creates the LVGL and UART tasks, and starts the main loop.
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

    // Create the command queue
    commandQueue = xQueueCreate(10, sizeof(char*));

    // Create the LVGL task and the UART RX task for FREERTOS and assign them to different cores
    xTaskCreatePinnedToCore(lvgl_task, NULL, 8 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(rx_task, "uart_rx_task", 1024*2, NULL, 4, NULL, 0);

    configure_backlight_PWM(1000, 10);          // Configure PWM and set frequency to 1 kHz with 50% duty cycle

    uart_write_bytes(UART_NUM_2, "UR\n", 3);    // Send a command to request the status of all UI parameters    
}