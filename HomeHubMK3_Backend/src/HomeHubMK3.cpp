/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mligh/OneDrive/HomeHubMK3/HomeHubMK3_Backend/src/HomeHubMK3.ino"
/*
 * Project HomeHubMK2
 * Description:
 * Author:
 * Date:
 */
#include "SPI.h"
#include "Sparkfun_DRV2605L.h"
uint16_t color565(uint8_t red, uint8_t green, uint8_t blue);
void updatePlug(int plugOn);
void updateBulb(int bulbOn, int bulbBrightness, int colorTemp);
void updateBulbColor(int bulbOn, int bulbBrightness, int colorTemp);
void setBulbs();
void setPlugs();
void addBulb(uint8_t deviceIP[4]);
void addPlug(uint8_t deviceIP[4]);
void pullSavedDevices(uint8_t baseDevIP[4]);
void pushSavedDevices();
void setColorTemperature(float kelvinTemp);
void convert_NB(float temperature, float brightness);
void _normalize();
static void processCommand(const char* input, uint16_t length);
void updateBulbs();
void updateAmbientBrightness();
void updateUI();
void sendAllParameters();
void setup();
void loop();
void updateSelectedItems();
bool updateBrightness();
bool updateColor();
void encoderUpdate();
void buttonUpdate();
void clickUpdate();
int brewCoffee(const char *command);
void BLEScan();
int setBulbColor(const char *command);
void updateEEPROM();
void resetColorTempFromEEPROM();
#line 9 "c:/Users/mligh/OneDrive/HomeHubMK3/HomeHubMK3_Backend/src/HomeHubMK3.ino"
#undef min
#undef max
#include <vector>

//#include "neopixel.h"
#include "tplink.h"

#include "HomeHubMK3.h"

///////////////////////////////////
////// HAPTIC CONFIGURATION ///////
///////////////////////////////////

//SFE_HMD_DRV2605L HMD;

/////////////////////////////////
////   LIGHT CONTROL MACROS  ////
/////////////////////////////////

    #define NLIGHT      11,4,1      //Night light pixel color

    #define NLIGHT_PXNUM    3       //Pixel number of night-light

/////////////////////////////////
//// SOFTWARE CONTROL MACROS ////
/////////////////////////////////

    #define FAN_MODE        1
    #define DEVICES_MODE    2
    #define CLOCK_MODE      3
    #define BRIGHTNESS_MODE 4
    #define COLOR_MODE      5
    #define SETTING_MODE    6    

    #define MAXMODE         6           //Maximum number of control modes
    #define MODESEL_TIMEOUT 20000       //Amount of time to show highlighter on screen after user stops moving the dial

    #define DEVICE_Y        36
    
    #define ONBRT           0.15        //Default starting brightness for auto
    #define ONTEMP          35          //Default starting temperature for auto
    #define ENDTEMP         25.1        //Default ending temperature for auto
    #define HRSTARTFADE     20          //8 PM
    #define HRENDFADE       22          //Hour to reach ending auto-temperature
    #define HRRESETCOLOR    6           //Number of hours after HRENDFADE to reset color temperature
    #define FADETIME_MIN    30          //Amount of time to fade to full brightness

/////////////////////////////////
///// EEPROM CONFIGURATION //////
/////////////////////////////////

    #define BT_MAXDEVICES       8           //Max number of storeable devices
    #define EEP_DEVCOUNT_REG    16          //Device eeprom max location, should be 2x BT_MAXDEVICES
    #define EEP_MODE_REG        17          //EEPROM location to hold last used mode
    #define EEP_BRT_REG         18          //EEPROM location to hold last used brightness value
    #define EEP_COL_REG         19          //EEPROM location to hold last used color temperature
    #define EEP_KASA_DEVS       20          //EEPROM location to hold number of Kasa devices

/////////////////////////////////
//// BLUETOOTH CONFIGURATION ////
/////////////////////////////////   

    #define BLE_SCAN_PD         2000        //Number of milliseconds between Bluetooth scans
    #define CLICK_DIS_SCAN      2000
    #define BT_BOUND            -65         //dBi of received signal to turn on lights
    #define BT_TIMEOUT_MS       180000      //Number of seconds before turning lights off
    #define BT_AUTO_RE_EN_MS    900000      //Number of milliseconds before re-enabling scanner after manual shutoff
    #define BT_BRT_AUTO_ON      100         //Minimum photoresistor value for automatic turn on of lights  
    #define SCAN_RESULT_COUNT   16          //Max number of devices to discover per scan
    #define CUSTOM_DATA_LEN     31           //Number of bytes to include in the advertising packet for BLE

/////////////////////////////////
///// HARDWARE CONFIGURATION ////
/////////////////////////////////

    #define TFT_DC  A1
    #define TFT_CS  A2
    #define TFT_RST D2
    #define TFT_BL  A5      //LCD backlight pin (PWM)
    #define BRTSNS  A0      // Photoresistor brightness sensing
    #define ENC_CLK D5
    #define ENC_DET D4
    #define ENC_BUT D3
    #define CAP_BUT D10

/////////////////////////////////
///// HAPTIC CONFIGURATION //////
/////////////////////////////////

    #define ROT_SLIDER_EFF  26      //DRV2605L effect to be played when rotating the dial
    #define BUT_CLICK_EFF   1       //DRV2605L effect to be played when touching the capacitive button
    #define ROT_END_EFF     1      //DRV2605L effect to be played when rotating the dial and at extremes of sliders

/////////////////////////////////
////// MENU GRAPHICS MACROS /////
/////////////////////////////////

    #define MENU_CHAR_COUNT     20      //Maximum width of characters inside of menu item
    #define MENU_STND_HEIGHT    50      //Pixel count of rectangle height for standard menu items
    #define MENU_TALL_HEIGHT    90      //Pixel count of rectangle height for large menu items
    #define MENU_PAD_SIZE       10      //Distance between menu items of both sizes
    #define MENU_TXT_PAD_SIZE   15      //Number of pixels between top of menu item box and cursor start for text
    #define MENU_ITEM_ROUND     15      //Radius in pixels of the rounding of rectangular menu items
    #define MENU_TALL_TXT_PAD   50      //Number of pixels between text lines in large menu items

/////////////////////////////////
/////// TP-Link Plug IPs ////////
/////////////////////////////////

void drawFastFilledBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg_color);
void drawFastBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);

uint8_t plugIP1[4] = {192,168,0,102};
uint8_t plugIP2[4] = {192,168,0,103};
uint8_t plugIP3[4] = {192,168,0,106};
uint8_t plugIP4[4] = {192,168,0,107};

uint8_t bulbIP1[4] = {192,168,0,100};   //KL-130
uint8_t bulbIP2[4] = {192,168,0,101};   //KL-130
uint8_t bulbIP3[4] = {192,168,0,105};   //KL-135

SYSTEM_MODE(AUTOMATIC);
STARTUP(WiFi.selectAntenna(ANT_INTERNAL)); // selects the u.FL antenna
SYSTEM_THREAD(DISABLED);

bool selectingMode = true;
int mode, lastMode;               //current operating mode of lamp
bool update;             //var to update sensor values
bool submenu;           //Flag set true if LCD will be displaying a submenu as opposed to main splash

int i;                  //loop vars
int j;

int photo;              //Variable to hold value of photoresistor sensor
int percentBrightness;
bool click;
int mprev;

uint16_t minsElapsed;
uint8_t startFadeHr;
float curColorTemperature, lastColorTemperature;
float dimPCT, lastDimPCT;          //Global to hold the dimming percentage from the slider
bool updatedColorFromUI = false; //Flag to tell if the color temperature was updated from the UI
bool sendParametersRequest = false; //Flag to tell if the parameters were requested from the UI
float targetColorTemperature;

//Bluetooth Variables
uint8_t numStoredDevices;               //Counter for number of saved devices
uint8_t storedKeys[BT_MAXDEVICES<<1];   //Array to hold byte keys for devices
uint8_t devRSSI[BT_MAXDEVICES];         //Array to hold RSSI for each device          
uint64_t lastHeard;                     //Time in milliseconds when a known device was heard
bool AutoBrightnessActive = false;      //Flag set true when the auto-on command is set to fade brightness up slowly
bool AutoColorRamping = false;          //Flag set true when the color temperature is automatically ramping down
bool AutoCTL;                           //Boolean to disable automatic shutoff when no bluetooth device is present
bool reEnOnDiscover;                    //Boolean to tell scanner to re-enable if device is found after a manual turnoff
bool scanEn = true;                     //Do bluetooth scanning
uint64_t buttonDebounce, scanInterval, encoderBounce;
bool updateMode = false;
bool modeOtherClick = false;
bool lightStatus = true;
bool lastLightStatus = false;
bool bulbShown = true;
uint64_t discoveredWatch = 0;
uint64_t discoveredPhone = 0;
uint64_t timeAutoOn = 0;
uint64_t capGhostTimer = 0;             //Don't update bulbs due to the capacitive button being pressed during some time because it ghost presses (Wi-Fi?)
bool watchIcon = false;
bool phoneIcon = false;
bool fanOn = false;
uint8_t clickEffectNum = ROT_SLIDER_EFF;
uint8_t *rotationVar;                   //Increment or decrement this in the encoder interrupt when rotated, use to navigate software menus
uint8_t rotationMax = 1;                //Change this value to set the maximum value the encoder interrupt will increment rotationVar to (inclusive)
uint8_t rotationMin = 0;

BleScanResult scanResults[SCAN_RESULT_COUNT];

// Hardware SPI on Feather or other boards
Timer eepromUpdate(30000,updateEEPROM);
Timer resetColorTemp(HRRESETCOLOR*3600000,resetColorTempFromEEPROM);

uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

class TPLinkDevice{
public:
    uint8_t deviceIP[4];
    uint8_t deviceType;
    TPLinkDevice(uint8_t devIP[4], uint8_t type){
        deviceType = type;
        for(uint8_t j = 0; j < 4; j++) deviceIP[j] = devIP[j];
    }
    void updatePlug(int plugOn){
        TPLink_Plug(deviceIP, plugOn);
    }
    void updateBulb(int bulbOn, int bulbBrightness, int colorTemp){
        TPLink_Bulb(deviceIP, bulbOn, bulbBrightness, colorTemp);
    }
};

class TPLinkController{
    public:
    int plugsOn = 0;
    int bulbsOn = 0;
    int bulbsBrightness = 0;
    int bulbsColor = 27;
    std::vector<TPLinkDevice> TPLinkDevices;
    void updateBulbColor(int bulbOn, int bulbBrightness, int colorTemp){
        bulbsOn = bulbOn;
        bulbsBrightness = bulbBrightness;
        bulbsColor = colorTemp;
    }
    void setBulbs(){
        for(TPLinkDevice dev: TPLinkDevices){
            Serial.printlnf("Set bulb %d %d", dev.deviceIP[3], dev.deviceType);
            if(dev.deviceType == 0) TPLink_Bulb(dev.deviceIP,bulbsOn,bulbsBrightness,bulbsColor);
        }
    }
    void setPlugs(){
        for(TPLinkDevice dev: TPLinkDevices){
            if(dev.deviceType == 1) TPLink_Plug(dev.deviceIP,plugsOn);
        }
    }
    void addBulb(uint8_t deviceIP[4]){
        TPLinkDevices.push_back(TPLinkDevice(deviceIP, 0));
        //Serial.printlnf("Created device with IP: %d", TPLinkDevices.back().deviceIP[3]);
    }
    void addPlug(uint8_t deviceIP[4]){
        TPLinkDevices.push_back(TPLinkDevice(deviceIP, 1));
    }
    void pullSavedDevices(uint8_t baseDevIP[4]){    //Read from EEPROM and populate vector of TPLink Devices, takes IP of the microcontroller
        TPLinkDevices.clear();
        uint8_t kasaDevCount = 0;
        uint8_t newDevIP[4];
        for(uint8_t k = 0; k < 3; k++) newDevIP[k] = baseDevIP[k];
        EEPROM.get(EEP_KASA_DEVS, kasaDevCount);
        //Serial.printlnf("Found %d devices stored.", kasaDevCount);
        for(uint16_t j = 0; j < kasaDevCount*2; j+=2){
            uint8_t devIP;
            uint8_t devType;
            EEPROM.get(EEP_KASA_DEVS + j + 1, devType);
            EEPROM.get(EEP_KASA_DEVS + j + 2, devIP);
            newDevIP[3] = devIP;
            if(devType == 0) addBulb(newDevIP);
            else if(devType == 1) addPlug(newDevIP);
            //Serial.printlnf("Pulled device with IP: %d", newDevIP[3]);
        }
    }
    void pushSavedDevices(){    //Take vector of TPLink Devices and overwrite stored memory
        uint8_t devIP;
        uint8_t devType;
        uint8_t devCount = TPLinkDevices.size();
        uint8_t iter = 1;
        EEPROM.put(EEP_KASA_DEVS,devCount);
        //Serial.printlnf("Stored %d devices.", devCount);
        for(TPLinkDevice dev: TPLinkDevices){
            devIP = dev.deviceIP[3];
            devType = dev.deviceType;
            EEPROM.put(EEP_KASA_DEVS + (iter++), devType);
            EEPROM.put(EEP_KASA_DEVS + (iter++), devIP);
            //Serial.printlnf("Stored device with IP: %d", devIP);
        }
    }
};

//RGBColor does the color calculations for the neopixels to implement the color temperature and brighness
class RGBColor{
public:
    int red, green, blue;
    float _brightness;
    float _temperature;
    void setColorTemperature(float kelvinTemp){
        //Red Calculation
        if(kelvinTemp <= 66){
	        red = 255;
        }
        else{
            red = kelvinTemp - 60;
            red = 329.698727446*pow(red,-0.1332047592);
            if(red < 0) red = 0;
            if(red > 255) red = 255; 
        }

        //Green Calculation
        if(kelvinTemp <= 66){
            green = 92*log(kelvinTemp) - 161.1195681661;
            if(green < 0) green = 0;
            if(green > 255) green = 255;
        }
        else{
            green = kelvinTemp-60;
            green = 247*pow(green,-0.0755148492);
            if(green < 0) green = 0;
            if(green > 255) green = 255;
        }

        //Blue Calculation
        if(kelvinTemp >= 66){
            blue = 255;
        }
        else{
            if(kelvinTemp <= 19){
                blue = 0;
            }
            else{
                blue = kelvinTemp - 10;
                blue = 125 * log(blue) - 305.0447927307;
                if(blue < 0) blue = 0;
                if(blue > 255) blue = 255;
            }
        }

        
    }
    void convert_NB(float temperature, float brightness){
      _temperature = constrain(temperature, 0, 65500);
      _brightness = constrain(brightness, 0, 100);
      float _green, _red, _blue;

      _red = _green = _blue = 0;
      float t = _temperature * 0.01;
      
      if (t <= 66)
      {
        _red = 255;
        _green = t - 2;
        _green = -155.25485562709179 - 0.44596950469579133 * _green + 104.49216199393888 * log(_green);
        _blue = 0;
        if (t > 20)
        {
          _blue = t - 10;
          _blue = -254.76935184120902 + 0.8274096064007395 * _blue + 115.67994401066147 * log(_blue);
        }
      }
      else
      {
        _red = t - 55.0;
        _red = 351.97690566805693 + 0.114206453784165 * _red - 40.25366309332127 * log(_red);
        _green = t - 50.0;
        _green = 325.4494125711974 + 0.07943456536662342 * _green - 28.0852963507957 * log(_green);
        _blue = 255;
      }

      red = (int)_red;
      green = (int)_green;
      blue = (int)_blue;
      
      _normalize();
    }
    void _normalize()
    {
      float f = 0.01 * _brightness;

      red   = constrain(f * red,   0, 255);
      green = constrain(f * green, 0, 255);
      blue  = constrain(f * blue,  0, 255);
    }
};

RGBColor currentColor;  //Declare a color object
TPLinkController TPLinkDeviceController;

/// @brief Processes a serial command from the User Interface
/// @param input String of the command
/// @param length Number of characters in the command
static void processCommand(const char* input, uint16_t length) {
    //First character is the node identifier (U = UI, M = Main Controller, S = Secondary Controller)
    //Second character is the command
    //Remaining characters are the payload. Process the data based on the type of command and where it's coming from

    if(length < 0) return; //Invalid length

    char payload[length];
    for(int i = 2; i < length; i++){
        payload[i-2] = input[i];
    }

    uint8_t targetBrightness = 0;
    uint8_t targetColor = 0;
    if(input[0] == 'U'){
        switch(input[1]){
            case 'B': //Device status update - Shows if one of the BLE devices is discovered
                targetBrightness = atoi(payload);
                if(targetBrightness > 100) targetBrightness = 100;
                dimPCT = targetBrightness / 100.0;
                updatedColorFromUI = true;  //Set this here so the controller doesn't immediately echo back to the UI
                update = true;
                break;
            case 'C': //Device status update - Shows if one of the BLE devices is discovered
                targetColor = atoi(payload);
                if(targetColor < 24) targetColor = 24;
                if(targetColor > 70) targetColor = 70;
                curColorTemperature = targetColor;
                updatedColorFromUI = true;  //Set this here so the controller doesn't immediately echo back to the UI
                update = true;
                break;
            case 'G':   //Toggle command. Toggles light status
                AutoCTL = false;
                update = true;
                AutoCTL = false;
                lightStatus = !lightStatus;
                break;
            case 'R':    //Request for all UI parameters
                sendParametersRequest = true;
            default:
                break;
        }
    }
}

void updateBulbs(){
    Serial.printlnf("Bulb Data Available: %d", DataAvailable());
    if(lightStatus == false && lastLightStatus){
        for(i=((1.0-dimPCT)*64.0);i<64;i++){
            TPLinkDeviceController.updateBulbColor(1,100-i*1.5625,curColorTemperature*100);
            TPLinkDeviceController.setBulbs();
            currentColor.convert_NB(curColorTemperature * 100, 100-i*1.5625);
            Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
            //delay(20);
        }
        TPLinkDeviceController.updateBulbColor(0,0,curColorTemperature*100);
        TPLinkDeviceController.setBulbs();
    }
    else if(lightStatus == true){ 
        //fillStrip(true, 64, currentColor.red,currentColor.green,currentColor.blue, 0, dimPCT);
        TPLinkDeviceController.updateBulbColor(1,(int)(dimPCT*100),curColorTemperature*100);
        TPLinkDeviceController.setBulbs();
        currentColor.convert_NB(curColorTemperature * 100, dimPCT*100);
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        delay(3);
    }
    else{
        TPLinkDeviceController.updateBulbColor(0,0,curColorTemperature*100);
        TPLinkDeviceController.setBulbs();
        Serial1.printlnf("MF%x %x %x", 0, 0, 0);
    }
}

void updateAmbientBrightness(){
    const int average_samples = 10; // Number of samples to average
    const int max_brightness = 2000;
    static int brightnessReadings[average_samples] = {0}; // Array to store the last 10 readings
    static int currentIndex = 0;            // Index to track the current position in the array
    static int sum = 0;                     // Sum of the readings
    static int average = 0;                 // Average of the readings
    static int lastBrightness = 0;             // Last calculated average

    // Read the brightness sensor value. 0-4095
    int currentReading = analogRead(BRTSNS);

    // Update the sum and replace the oldest reading
    sum -= brightnessReadings[currentIndex];
    brightnessReadings[currentIndex] = currentReading;
    sum += currentReading;

    // Update the current index (circular buffer)
    currentIndex = (currentIndex + 1) % average_samples;

    // Calculate the new average
    average = sum / average_samples;

    percentBrightness = map(average, 0, max_brightness, 0, 100);
    if(percentBrightness >= 100) percentBrightness = 100;

    // Check if the average has changed by more than 3%
    if (abs(percentBrightness - lastBrightness) >= 3) {
        // Transmit the new average over Serial1
        
        Serial1.printlnf("ML%d", percentBrightness);
        lastBrightness = percentBrightness; // Update the last average
    }
}

void updateUI(){

    if(phoneIcon != (System.millis() - discoveredPhone < BT_TIMEOUT_MS)){
        phoneIcon = !phoneIcon;
        Serial1.printlnf(phoneIcon ? "MP1" : "MP0");
    }

    if(watchIcon != (System.millis() - discoveredWatch < BT_TIMEOUT_MS)){
        watchIcon = !watchIcon;
        Serial1.printlnf(watchIcon ? "MW1" : "MW0");
    }

    if(mprev != Time.minute()){
        mprev = Time.minute();
        Serial1.printlnf("MT%d %d", Time.hourFormat12(), Time.minute());
    }

}

void sendAllParameters(){
    if(sendParametersRequest){
        Serial1.printlnf("MP%d", phoneIcon ? 1 : 0);
        Serial1.printlnf("MW%d", watchIcon ? 1 : 0);
        Serial1.printlnf("ML%d", percentBrightness);
        Serial1.printlnf("MB%d", (int)(dimPCT*100));
        Serial1.printlnf("MC%d", curColorTemperature);
        Serial1.printlnf("MT%d %d", Time.hourFormat12(), Time.minute());

        currentColor.convert_NB(curColorTemperature * 100, dimPCT * 100);
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        sendParametersRequest = false;
    }
    
}

void setup() {
    //WiFi.setCredentials("p0cK3T-r0uT3R", "NC$U3ngineering");

    Serial.begin(115200);
    Serial1.begin(115200);
    Serial1.printlnf("TEST");
    WiFi.selectAntenna(ANT_INTERNAL);
    
    BLE.on();
    //BLE.selectAntenna(BLE_ANT_EXTERNAL);
    BLE.setScanTimeout(50);

    Particle.function("lightControl", brewCoffee);
    Particle.function("SetLightColor", setBulbColor);
    Particle.variable("Light Sensor", photo);
    //Particle.variable("Color Temperature", curColorTemperature);
    
    pinMode(CAP_BUT, INPUT_PULLDOWN);
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST,OUTPUT);
    pinMode(ENC_CLK, INPUT);
    pinMode(ENC_DET, INPUT);
    pinMode(TFT_BL,OUTPUT);
    pinMode(ENC_BUT,INPUT_PULLDOWN);
    pinMode(BRTSNS, INPUT);
    digitalWrite(TFT_CS,HIGH);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);
    attachInterrupt(ENC_CLK, encoderUpdate, CHANGE);
    attachInterrupt(ENC_BUT,clickUpdate,RISING);
    attachInterrupt(CAP_BUT,buttonUpdate,CHANGE);

    //EEPROM.write(EEP_MODE_REG,(uint8_t)1);
    EEPROM.write(EEP_BRT_REG,(uint8_t)100);
    EEPROM.write(EEP_COL_REG,(uint8_t)25);

    for(int k = 0; k < (BT_MAXDEVICES>>1); k++) EEPROM.write(i,0);
    EEPROM.write(EEP_DEVCOUNT_REG,2);

    EEPROM.write(0,7);
    EEPROM.write(1,31);
    EEPROM.write(2,5);
    EEPROM.write(3,152);

    EEPROM.get(EEP_DEVCOUNT_REG, numStoredDevices);          //Populate device count from EEPROM
    for(int k = 0; k < (numStoredDevices << 1); k++){     //Populate data keys from EEPROM
        EEPROM.get(k,storedKeys[k]);

    }
    for(int k = 0; k < numStoredDevices; k++){          //Set device RSSI's
        devRSSI[k] = 65;
    }
    devRSSI[1] = 100;
    scanEn = true;

    lastHeard = 0;
    scanInterval = System.millis();
    AutoCTL = true;
    reEnOnDiscover = false;

    photo = analogRead(BRTSNS);
    analogWrite(TFT_BL,photo >> 4);

    mode = 1;

    update = true;
    submenu = false;
    buttonDebounce = System.millis();

    //stripa.begin();
    //stripa.setPixelColor(NLIGHT_PXNUM,NLIGHT);
    //stripa.show();

    

    if(digitalRead(CAP_BUT)==LOW){
        Serial1.printlnf("PARTICLE CONNECT");
        Particle.connect();
        while(WiFi.connecting()){

            //delay(5);
        }
        while(!Particle.connected()){

        }
        //WiFi.localIP().toString().c_str()
        delay(1000);
        //BLE.on();
    }

    Serial1.printlnf("PARTICLE CONNECTED");

    curColorTemperature = EEPROM.read(EEP_COL_REG);
    lastColorTemperature = 0;
    dimPCT = EEPROM.read(EEP_BRT_REG);
    dimPCT /= 100.0;
    lastDimPCT = 0.0;
    if(dimPCT > 1) dimPCT = 1;
    if(curColorTemperature < 24) curColorTemperature = 24;
    if(curColorTemperature > 70) curColorTemperature = 70;
    encoderBounce = 0;

    //curColorTemperature = EEPROM.read(EEP_BRT_REG);
    //curStripBrightness = EEPROM.read(EEP_COL_REG)/100.0;
    //currentColor.setColorTemperature(curColorTemperature);
    
    RGB.control(true);
    RGB.color(0,0,0);

    Time.zone(-4);
    mprev = -1;

    //FOR /L %i IN (1,1,254) DO @ping -n 1 -w 200 192.168.1.%i | FIND /i "TTL"

    TPLinkDeviceController.addBulb(bulbIP1);
    TPLinkDeviceController.addBulb(bulbIP2);
    TPLinkDeviceController.addBulb(bulbIP3);
    TPLinkDeviceController.pushSavedDevices();

    Serial1.printlnf("SETUP DONE");

    //IPAddress localIP = WiFi.localIP();
    //uint8_t ipBytes[4] = {localIP[0], localIP[1], localIP[2], localIP[3]};
    //TPLinkDeviceController.pullSavedDevices(ipBytes);
      
    //analogWrite(A5,0);

    /*HMD.begin();
    delay(10);
    HMD.Mode(0);
    HMD.MotorSelect(0x63); //0x63
    HMD.Library(7); //change to 6 for LRA motors 
    updateSelectedItems();
    HMD.Waveform(0, 42); //56
    HMD.go();*/

    //digitalWrite(D7, HIGH);


}

void loop() {
    /////////////////////////////////////////////////
    //////////  Update Sensor Variables  ////////////
    /////////////////////////////////////////////////
    updateAmbientBrightness();
    sendAllParameters();
    
    /////////////////////////////////////////////////
    //////////  MAIN MENU DISPLAY UPDATE  ///////////
    /////////////////////////////////////////////////
    updateUI();

    if(Serial1.available()){
        char input[32];
        int length = Serial1.readBytesUntil('\n', input, sizeof(input));
        if(length > 0){
            processCommand(input, length);
        }
    }

    if(!submenu){
        if(lightStatus == false && AutoCTL){      //If lamp is currently off
            if(System.millis()-lastHeard < BT_TIMEOUT_MS && analogRead(BRTSNS) < BT_BRT_AUTO_ON){   //Check that the brightness sensor is dark so doesn't turn on during the day
                Log.info("Automatic-ON");
                AutoColorRamping = true;
                lightStatus = true;
                update = true;
            }
        }
        else if(AutoCTL){ // If lamp is on, turn of iff timed out
            if(System.millis()-lastHeard > BT_TIMEOUT_MS){
                Log.info("Automatic-OFF");
                lightStatus = false;
                AutoBrightnessActive = false;
                update = true;
            }
        }
        else if(!reEnOnDiscover){
            if(System.millis()-lastHeard > BT_AUTO_RE_EN_MS){  //Re-enable scanner if device has disappeared for a longer period of time
                Log.info("Manually on, looking for devices...");
                reEnOnDiscover = true;
            }
        }
        
        

        ///////////////////////////////////////////////////
        //  Automatic Brightness and Temperature Control //
        ///////////////////////////////////////////////////
        if(AutoBrightnessActive && System.millis() - timeAutoOn > 30000 && lightStatus){
            if(minsElapsed < FADETIME_MIN*2){
                if(dimPCT < 1){
                    dimPCT += (double)((1.0-ONBRT)/(FADETIME_MIN*2));
                    if(dimPCT > 1) dimPCT = 1;
                }
                //if(curColorTemperature > targetColorTemperature){   //Temperature ramp-up on auto-on
                //    curColorTemperature  -= (double)((ONTEMP-targetColorTemperature)/(FADETIME_MIN*2.0));
                //    if(curColorTemperature < targetColorTemperature) curColorTemperature = targetColorTemperature;
                //}
            }
            else{
                AutoBrightnessActive = false;
                AutoColorRamping = true;
                targetColorTemperature = ENDTEMP;
            }
            update = true;
            minsElapsed++;
            timeAutoOn = System.millis();
        }
        
        if(AutoColorRamping && HRENDFADE > HRSTARTFADE && Time.hour() >= HRSTARTFADE && lightStatus){// && Time.hour() < HRENDFADE){  //Not fading up in brightness. Automatically warm color temperature over time.
            if(Time.hour() < HRENDFADE && System.millis() - timeAutoOn > 30000){    //While in the ramp-down period
                if(AutoColorRamping && curColorTemperature > ENDTEMP){
                    float diff = ((((double)(HRENDFADE - Time.hour()) - ((double)Time.minute()/60.0)) / (double)(HRENDFADE - HRSTARTFADE)) * ((double)EEPROM.read(EEP_COL_REG) - ENDTEMP));
                    if(diff < 0) diff = 0;
                    curColorTemperature = diff + ENDTEMP;   //Linear temperature based on time between start and end of fading period
                    Serial.printlnf("New Temp: %f Num: %f", curColorTemperature, ((double)(HRENDFADE - Time.hour()) - ((double)Time.minute()/60.0)));
                }
                else{   //Reached end of ramp-down period
                    AutoColorRamping = false;
                    resetColorTemp.start(); //Now that the color temperature has reached end of ramp-down, start timer which will reset the color temperature for the morning
                    Serial.println("Ending Auto Color Temperature");
                }
                update = true;
                timeAutoOn = System.millis();
            }
        }

        //if(Time.hour() == HRRESETCOLOR && AutoColorRamping) curColorTemperature = EEPROM.read(EEP_COL_REG);

        if(mode && System.millis() - encoderBounce > MODESEL_TIMEOUT && selectingMode){
            mode = 0;
            update = true;
        }

        if(System.millis() - scanInterval > BLE_SCAN_PD && System.millis() - encoderBounce > CLICK_DIS_SCAN && scanEn){
            scanInterval = System.millis();
            BLEScan();
        }

        if(update){
            if(click){
                click = false;
                //HMD.stop();
                //HMD.Waveform(0, clickEffectNum); //9
                //HMD.go();
            }
            update = false;
            if(updateBrightness() || updateColor() || lightStatus != lastLightStatus){
                updateBulbs();
                lastLightStatus = lightStatus;
            }
            if(!submenu){   //Check that this action didn't enter a submenu
                updateSelectedItems();
                uint16_t bulbBorderColor = 0xFFFF;
                if(AutoBrightnessActive) bulbBorderColor = 0x07E0;
                else if(AutoColorRamping) bulbBorderColor = 0x79FF;
                //drawFastBitmap(106,DEVICE_Y,bulbframeglyph,27,40,bulbBorderColor);
            }
        }
    }

    /////////////////////////////////////////////////
    //////////  SUB MENU DISPLAY UPDATE  ////////////
    /////////////////////////////////////////////////
    else{
        if(update){
            if(click){
                click = false;
                //HMD.stop();
                //HMD.Waveform(0, clickEffectNum); //9
                //HMD.go();
            }
            if(mode == DEVICES_MODE){

            }
            else if(mode == CLOCK_MODE){

            }
        }
    }


    if(click){
        click = false;
        //HMD.stop();
        //HMD.Waveform(0, clickEffectNum); //9
        //HMD.go();
    }

    delay(10);
}

void updateSelectedItems(){
    
    if(mode != lastMode){
        lastMode = mode;
        //Serial.printlnf("New Mode: %d", mode);
    }
}

bool updateBrightness(){
    if(dimPCT != lastDimPCT){
        lastDimPCT = dimPCT;
        //EEPROM.write(EEP_COL_REG,(uint8_t)(dimPCT*100));
        currentColor.convert_NB(curColorTemperature * 100, dimPCT * 100);
        if(!updatedColorFromUI) Serial1.printlnf("MB%d", (int)(dimPCT*100));
        updatedColorFromUI = false;
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        return true;
    }
    return false;
}

bool updateColor(){
    if(curColorTemperature != lastColorTemperature){
        lastColorTemperature = curColorTemperature;
        //EEPROM.write(EEP_BRT_REG,curColorTemperature);
        currentColor.convert_NB(curColorTemperature * 100, dimPCT * 100);
        if(!updatedColorFromUI) Serial1.printlnf("MC%d", (int)curColorTemperature);
        updatedColorFromUI = false;
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        return true;
    }
    return false;
}

void encoderUpdate(){
    encoderBounce = System.millis();
    modeOtherClick = !modeOtherClick;
    if(digitalRead(ENC_CLK) != digitalRead(ENC_DET)){   //Clockwise
        if(selectingMode){
            if(!submenu && mode < MAXMODE && modeOtherClick && mode == lastMode){
                mode++;
                click = true;
                clickEffectNum = ROT_SLIDER_EFF;
            }
            else if(submenu){
                if(*rotationVar < rotationMax) *rotationVar++;
            }
        }
        else{
            click = true;
            eepromUpdate.startFromISR();
            if(mode == BRIGHTNESS_MODE){
                if(dimPCT >= 0.97) clickEffectNum = ROT_END_EFF;
                else clickEffectNum = ROT_SLIDER_EFF;
                dimPCT += 0.03;
                if(dimPCT > 1.0){
                    dimPCT = 1.0;
                    click = false;
                }
            }
            else if(mode == COLOR_MODE){
                AutoColorRamping = false;
                if(curColorTemperature >= 89.5) clickEffectNum = ROT_END_EFF;
                else clickEffectNum = ROT_SLIDER_EFF;
                curColorTemperature += 0.5;
                if(curColorTemperature > 90){
                    curColorTemperature = 90;
                    click = false;
                }
            }
        }
    }
    else{       //Counter-clockwise
        if(selectingMode){
            if(!submenu && mode > 1 && modeOtherClick && mode == lastMode){
                mode--;
                click = true;
                clickEffectNum = ROT_SLIDER_EFF;
            }
            else if(submenu){
                if(*rotationVar > rotationMin) *rotationVar--;
            }
        }
        else{
            click = true;
            eepromUpdate.startFromISR();
            if(mode == BRIGHTNESS_MODE){
                if(dimPCT <= 0.03) clickEffectNum = ROT_END_EFF;
                else clickEffectNum = ROT_SLIDER_EFF;
                dimPCT -= 0.03;
                if(dimPCT < 0.0){
                    dimPCT = 0.0;
                    click = false;
                }
            }
            else if(mode == COLOR_MODE){
                AutoColorRamping = false;
                if(curColorTemperature <= 0.5) clickEffectNum = ROT_END_EFF;
                else clickEffectNum = ROT_SLIDER_EFF;
                curColorTemperature -= 0.5;
                if(curColorTemperature < 25){
                    curColorTemperature = 25;
                    click = false;
                }
            }
        }
    }
    update = true;
}

void buttonUpdate(){
    encoderBounce = System.millis();
    if(System.millis() - buttonDebounce  < 200 || System.millis() - capGhostTimer < 2000) return;
    buttonDebounce = System.millis();
    click = true;
    clickEffectNum = BUT_CLICK_EFF;
    AutoBrightnessActive = false;
    if(!digitalRead(CAP_BUT)){
        update = true;
        AutoCTL = false;
        lightStatus = !lightStatus;
        if(!lightStatus) capGhostTimer = System.millis();
    }
}

void clickUpdate(){
    if(System.millis() - buttonDebounce < 75) return;
    buttonDebounce = System.millis();
    AutoBrightnessActive = false;
    update = true;
    modeOtherClick = false;
    selectingMode = !selectingMode;
}

int brewCoffee(const char *command)
{
    if(strcmp(command,"toggle")==0){    
        AutoCTL = false;
        update = true;
        AutoCTL = false;
        lightStatus = !lightStatus;
    }
    else if(strcmp(command,"evoff")==0){
        AutoCTL = false;
        update = true;
        lightStatus = false;
        AutoBrightnessActive = false;
        capGhostTimer = System.millis();
    }
    else if(strcmp(command,"evon")==0){
        AutoCTL = false;
        update = true;
        lightStatus = true;
        AutoBrightnessActive = false;
    }
    else if(strcmp(command,"autoon")==0){
        AutoBrightnessActive = true;
        minsElapsed = 0;
        startFadeHr = Time.hour()+1;
        timeAutoOn = System.millis() + 30000;
        targetColorTemperature = ONTEMP;
        dimPCT = ONBRT;
        curColorTemperature = EEPROM.read(EEP_COL_REG);
        update = true;
    }
    else if(strcmp(command,"autocolor")==0){
        if(AutoColorRamping){
            AutoColorRamping = false;
            curColorTemperature = EEPROM.read(EEP_COL_REG);
        } 
        else{
            AutoColorRamping = true;
        }
        update = true;
    }
    else if(strcmp(command,"bton")==0){
        scanEn = true;
    }
    else if(strcmp(command,"btoff")==0){
        scanEn = false;
    }
    else if(strcmp(command,"comon")==0){
        TPLink_Plug(plugIP3, 1);
        TPLink_Plug(plugIP4, 1);
    }
    else if(strcmp(command,"comoff")==0){
        TPLink_Plug(plugIP3, 0);
        TPLink_Plug(plugIP4, 0);
    }
    else if(strcmp(command,"plg1on")==0){
        TPLink_Plug(plugIP1, 1);
    }
    else if(strcmp(command,"plg2on")==0){
        TPLink_Plug(plugIP2, 1);
    }
    else if(strcmp(command,"plg3on")==0){
        TPLink_Plug(plugIP3, 1);
    }
    else if(strcmp(command,"plg4on")==0){
        TPLink_Plug(plugIP4, 1);
    }
    else if(strcmp(command,"plg1off")==0){
        TPLink_Plug(plugIP1, 0);
    }
    else if(strcmp(command,"plg2off")==0){
        TPLink_Plug(plugIP2, 0);
    }
    else if(strcmp(command,"plg3off")==0){
        TPLink_Plug(plugIP3, 0);
    }
    else if(strcmp(command,"plg4off")==0){
        TPLink_Plug(plugIP4, 0);
    }
    else if(strcmp(command,"pct25")==0){
        dimPCT = 0.25;
        update = true;
        lightStatus = true;
    }
    else if(strcmp(command,"pct50")==0){
        dimPCT = 0.50;
        update = true;
        lightStatus = true;
    }
    else if(strcmp(command,"pct75")==0){
        dimPCT = 0.75;
        update = true;
        lightStatus = true;
    }
    else if(strcmp(command,"pct100")==0){
        dimPCT = 1;
        update = true;
        lightStatus = true;
    }
    return 1;
}

void BLEScan(){        //Function to scan for nearby bots advertising over BLE or connect to a specific one.
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);                                //Get number of found BLE devices
	if (count > 0) {                                                                        //Check if any were found
		for (uint8_t ii = 0; ii < count; ii++) {                                            //Loop over discovered bots
			//if(scanResults[ii].rssi() < 0 && scanResults[ii].rssi() > -45){
                uint8_t bufName[32];                                           //Array to hold the bytes of custom data
                size_t cDataLen = scanResults[ii].advertisingData().customData(bufName,CUSTOM_DATA_LEN);      //Retrieve the custom data from the advertisement
                //if(cDataLen < 8 || cDataLen > 12) continue;
                //Serial.printlnf("Custom data len: %d",cDataLen);
                //uint8_t advData[32];                                                        //Buffer that gets populated with BLE advertising data
                //size_t aDataLen = scanResults[ii].advertisingData().get(advData,CUSTOM_DATA_LEN);                          //Populate the buffer
                //Serial.printlnf("RSSI: %d    DATA: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",scanResults[ii].rssi(), advData[0], advData[1],advData[2],advData[3],advData[4], advData[5],advData[6],advData[7], bufName[0], bufName[1], bufName[2], bufName[3], bufName[4], bufName[5], bufName[6], bufName[7], bufName[8], bufName[9], bufName[10],bufName[11],bufName[12],bufName[13], bufName[14], bufName[15], bufName[16], bufName[17], bufName[18], bufName[19], bufName[20], bufName[21], bufName[22], bufName[23], bufName[24],bufName[25],bufName[26],bufName[27]);
                //if(bufName[0] == 76) Serial.printlnf("Found an iDevice %d %d", bufName[3], bufName[5]);
                uint8_t deviceNumber;                                                       //Temporary variable to hold the device numbers based on the EEPROM retrieved value
                for(deviceNumber = 0; deviceNumber < numStoredDevices; deviceNumber++){     //Loop over the devices saved in memory
                    if(bufName[0] == 76 && bufName[3] == storedKeys[deviceNumber << 1] && bufName[5] == storedKeys[(deviceNumber << 1)+1] && scanResults[ii].rssi() > (0-devRSSI[deviceNumber])){   //Check if this device matches any of the saved devices
                        //Serial.printlnf("Found Device IDs: %d %d, RSSI: %d", storedKeys[deviceNumber << 1], storedKeys[(deviceNumber << 1)+1], scanResults[ii].rssi());                             
                        lastHeard = System.millis();                                        //Save the time that a saved device was last seen, if too much time elapses since this time was saved, then turn off the lights
                        if(devRSSI[deviceNumber] < 70) discoveredPhone = System.millis();   //Strong RSSI devices are phones, update the phone update time for updating the phone icon
                        else discoveredWatch = System.millis();                             //Strong RSSI devices are watches, update the phone update time for updating the watch icon
                        if(reEnOnDiscover){
                            AutoCTL = true;
                            reEnOnDiscover = false;
                            //Serial.printlnf("Found device, re-enabling automatic control");
                       }
                    }
                }
            //}
        }
	}
}

int setBulbColor(const char *command){
    int receivedVal;
    receivedVal = atoi(command);
    int RedVal = receivedVal%1000;
    int GreenVal = (receivedVal/1000)%1000;
    int BlueVal = (receivedVal/1000000)%1000;
    TPLink_RGB_Bulb(bulbIP2, 1, RedVal, GreenVal, BlueVal);
    return 1;
}

void updateEEPROM(){
    EEPROM.write(EEP_BRT_REG,(uint8_t)(dimPCT*100.0));
    EEPROM.write(EEP_COL_REG,(uint8_t)curColorTemperature);
    eepromUpdate.stopFromISR();
}

void resetColorTempFromEEPROM(){
    curColorTemperature = EEPROM.read(EEP_COL_REG);
    resetColorTemp.stopFromISR();   //Only run once
}
