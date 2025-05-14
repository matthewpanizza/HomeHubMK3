/*
 * Project HomeHubMK3 Backend
 * Description: Project to automatically control TP-Link Kasa smart bulbs based on iOS device BLE proximity
 * Author: Matthew Panizza
 * Date: May 12, 2025. Legacy HomeHubMK2 authored September 2022.
 */
#include "SPI.h"
#include "Sparkfun_DRV2605L.h"
#undef min
#undef max
#include <vector>
#include "tplink.h"
#include "HomeHubMK3.h"

/////////////////////////////////
//// SOFTWARE CONTROL MACROS ////
/////////////////////////////////  
    
    #define ONBRT           0.15        // Default starting brightness for auto
    #define ONTEMP          35          // Default starting temperature for auto
    #define ENDTEMP         25.1        // Default ending temperature for auto
    #define HRSTARTFADE     20          // 8 PM
    #define HRENDFADE       22          // Hour to reach ending auto-temperature
    #define HRRESETCOLOR    6           // Number of hours after HRENDFADE to reset color temperature
    #define FADETIME_MIN    30          // Amount of time to fade to full brightness

/////////////////////////////////
///// EEPROM CONFIGURATION //////
/////////////////////////////////

    #define BT_MAXDEVICES       8       // Max number of storeable devices
    #define EEP_DEVCOUNT_REG    16      // Device eeprom max location, should be 2x BT_MAXDEVICES
    #define EEP_MODE_REG        17      // EEPROM location to hold last used mode
    #define EEP_BRT_REG         18      // EEPROM location to hold last used brightness value
    #define EEP_COL_REG         19      // EEPROM location to hold last used color temperature
    #define EEP_KASA_DEVS       20      // EEPROM location to hold number of Kasa devices

/////////////////////////////////
//// BLUETOOTH CONFIGURATION ////
/////////////////////////////////   

    #define BLE_SCAN_PD         2000    // Number of milliseconds between Bluetooth scans
    #define CLICK_DIS_SCAN      2000
    #define BT_BOUND            -65     // dBi of received signal to turn on lights
    #define BT_TIMEOUT_MS       180000  // Number of seconds before turning lights off
    #define BT_AUTO_RE_EN_MS    900000  // Number of milliseconds before re-enabling scanner after manual shutoff
    #define BT_BRT_AUTO_ON      100     // Minimum photoresistor value for automatic turn on of lights  
    #define SCAN_RESULT_COUNT   16      // Max number of devices to discover per scan
    #define CUSTOM_DATA_LEN     31      // Number of bytes to include in the advertising packet for BLE

/////////////////////////////////
///// HARDWARE CONFIGURATION ////
/////////////////////////////////

    #define BRTSNS  A0                  // Photoresistor brightness sensing

/////////////////////////////////
///// HAPTIC CONFIGURATION //////
/////////////////////////////////

    //SFE_HMD_DRV2605L HMD;
    #define ROT_SLIDER_EFF  26          // DRV2605L effect to be played when rotating the dial
    #define BUT_CLICK_EFF   1           // DRV2605L effect to be played when touching the capacitive button
    #define ROT_END_EFF     1           // DRV2605L effect to be played when rotating the dial and at extremes of sliders

/////////////////////////////////
/////// TP-Link Plug IPs ////////
/////////////////////////////////

uint8_t plugIP1[4] = {192,168,0,102};
uint8_t plugIP2[4] = {192,168,0,103};
uint8_t plugIP3[4] = {192,168,0,106};
uint8_t plugIP4[4] = {192,168,0,107};

uint8_t bulbIP1[4] = {192,168,0,100};   // KL-130
uint8_t bulbIP2[4] = {192,168,0,101};   // KL-130
uint8_t bulbIP3[4] = {192,168,0,105};   // KL-135

SYSTEM_MODE(AUTOMATIC);          
STARTUP(WiFi.selectAntenna(ANT_INTERNAL)); // selects the u.FL antenna
SYSTEM_THREAD(DISABLED);

//Logging class
SerialLogHandler logHandler;

//Bluetooth Variables
uint8_t storedBLEDeviceCount;               //Counter for number of saved devices
uint8_t storedBLEKeys[BT_MAXDEVICES * 2];   //Each BLE device has its advertising data matched with 2 bytes. This array contains the bytes for the devices we wish to scan for. Fetched from EEPROM
uint8_t devRSSI[BT_MAXDEVICES];             //Array to hold RSSI for the devices we wish to scan for   
uint64_t lastBLEDetectionTime;              //Time in milliseconds when a known device was heard. Compared against the millisecond counter to automatically control lights
uint64_t lastBLEScanTime;                   //Time in milliseconds when a BLE scan was done. Used to limit number of scans per second.
bool doBLERangeScanning = true;             //Flag to control if lights should turn on/off based on BLE device proximity

//Device presence flags
uint64_t discoveredWatchTimestamp = 0;      //Time in milliseconds when an apple watch device was last discovered from the BLE scan
uint64_t discoveredPhoneTimestamp = 0;      //Time in milliseconds when an iPhone device was last discovered from the BLE scan
bool watchPresenceIcon = false;             //Flag to indicate if the scanning mechanism thinks a watch has been discovered. Gets shown on UI with glyph
bool phonePresenceIcon = false;             //Flag to indicate if the scanning mechanism thinks an iPhone has been discovered. Gets shown on UI with glyph

//Light control variables
bool smartBulbState = true;                 //Flag indicating if the smart bulbs should be in the on state. Use this flag to maintain the last brightness percentage when turning off lights.
bool lastSmartBulbState = false;            //Flag indicating the on/off state of the smart bulbs last time an update was sent to them.
float currentColorTemperature;              //Kelvin color temperature of the smart bulbs. Ranges from 24.0 - 70.0.
float lastColorTemperature = 0;             //Kelvin color temperature of the smart bulbs last time they were updated. Use this to compare to currentColorTemperature.
float targetColorTemperature;               //When in automatic color fade mode, this is the target Kelvin color temperature at the end of the fade
float currentBulbBrightness;                //Brightness percentage of the smart bulbs. Ranges from 0.0 - 1.0
float lastBulbBrightness = 0.0;             //Brightness percentage of the smart bulbs last time they were updated. Use this to compare to currentBulbBrightness.    

//Automatic light control variables
bool autoBrightnessActive = false;          //Flag set true when the auto-on command is set to fade brightness up slowly
bool autoColorRamping = false;              //Flag set true when the color temperature is automatically ramping down
bool autoControlLights;                     //Boolean to disable automatic shutoff when no bluetooth device is present
uint64_t autoOnTimestamp = 0;           
bool reEnableAutoControl;                   //Boolean to tell scanner to re-enable if device is found after a manual turnoff
uint16_t fadeUpMinutesElapsed;              //When in an automatic brightness fade-up, this is the number of minutes since the fade-up started
uint8_t fadeUpStartHour;                    //When in an automatic brightness fade-up, this is the hour number (24-hr format) we started fading up at

//Frontend control variables
bool updatedColorFromUI = false;            //Flag to tell if the color temperature was updated from the UI
bool sendParametersRequest = false;         //Flag to tell if the parameters were requested from the UI
int lcdBacklightPercentage;                 //Backlight percentage of the LCD on the frontend controller.
uint32_t bulbBorderColor;                   //Bulb border color (RGB888). Used to indicate special light control modes (auto brightness/color temperature)

BleScanResult scanResults[SCAN_RESULT_COUNT];

// Hardware SPI on Feather or other boards
Timer eepromUpdate(30000,updateEEPROM);

/// @brief Converts RGB values to 16-bit color565 format
uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

/// @brief Class to represent a TP link device (smart plug or bulb)
class TPLinkDevice{
public:
    uint8_t deviceIP[4];    // Array to represent the IP address of this device
    uint8_t deviceType;     // Device type (0 = bulb, 1 = plug)
    // Initializer for TP Link Device. Takes the IP address and device type
    TPLinkDevice(uint8_t devIP[4], uint8_t type){
        deviceType = type;
        for(uint8_t j = 0; j < 4; j++) deviceIP[j] = devIP[j];
    }
    // Function to change the state of a plug (0 = off, 1 = on)
    void updatePlug(int plugOn){
        TPLink_Plug(deviceIP, plugOn);
    }
    // Function to change the state, brightness, and color temperature of a bulb
    void updateBulb(int bulbOn, int bulbBrightness, int colorTemp){
        TPLink_Bulb(deviceIP, bulbOn, bulbBrightness, colorTemp);
    }
};

/// @brief Class to handle control of all TP Link devices. Holds a dictionary of devices and has helper functions to change states.
class TPLinkController{
    public:
    int plugsOn = 0;                            // Indicates if the plugs are on or off
    int bulbsOn = 0;                            // Indicates if the bulbs are on or off
    float bulbsBrightness = 0;                    // Current bulb brightness percentage (0.0 1.0)
    float bulbsColor = 27;                        // Current bulb color temperature (24-70)
    std::vector<TPLinkDevice> TPLinkDevices;    // List of all devices in the system
    /// @brief Function to update the state of all bulbs to a given brightness and color temperature
    void updateBulbColor(int bulbOn, float bulbBrightness, float colorTemp){
        bulbsOn = bulbOn;
        bulbsBrightness = bulbBrightness;
        bulbsColor = colorTemp;
    }
    /// @brief Function to apply the current brightness and color temperature to all bulbs
    void setBulbs(){
        // Loop over all devices and apply the brightness and color temperature to the bulbs
        for(TPLinkDevice dev: TPLinkDevices){
            //Log.info("Set bulb with IP [%d.%d.%d.%d] of type: %d", dev.deviceIP[0], dev.deviceIP[1], dev.deviceIP[2], dev.deviceIP[3], dev.deviceType);
            if(dev.deviceType == 0) TPLink_Bulb(dev.deviceIP, bulbsOn, bulbsBrightness * 100, bulbsColor * 100);
        }
    }
    /// @brief Function to apply the current on/off state to all plugs
    void setPlugs(){
        // Loop over all devices and apply the on/off state to the plugs
        for(TPLinkDevice dev: TPLinkDevices){
            if(dev.deviceType == 1) TPLink_Plug(dev.deviceIP, plugsOn);
        }
    }
    /// @brief Function to add a new bulb to the list of devices
    /// @param deviceIP IP address of the new bulb (i.e. {192,168,0,100})
    void addBulb(uint8_t deviceIP[4]){
        TPLinkDevices.push_back(TPLinkDevice(deviceIP, 0));
        //Serial.printlnf("Created device with IP: %d", TPLinkDevices.back().deviceIP[3]);
    }
    /// @brief Function to add a new plug to the list of devices
    /// @param deviceIP IP address of the new plug (i.e. {192,168,0,100})
    void addPlug(uint8_t deviceIP[4]){
        TPLinkDevices.push_back(TPLinkDevice(deviceIP, 1));
    }
    /// @brief Function to retrieve the list of devices from EEPROM and populate the vector of TPLinkDevices
    /// @param baseDevIP Gets the first three bytes of the IP address of the microcontroller (i.e. {192,168,0})
    void pullSavedDevices(uint8_t baseDevIP[4]){    // Read from EEPROM and populate vector of TPLink Devices, takes IP of the microcontroller
        /* EXAMPLE EEPROM
        Address   Value
          20:       3       // Number of stored devices (3)
          21:       0       // Device type of device 0 (bulb)
          22:       26      // Fourth byte of the IP Address ({192,168,0,26})
          21:       0       // Device type of device 0 (bulb)
          22:       28      // Fourth byte of the IP Address ({192,168,0,28})
          21:       1       // Device type of device 1 (plug)
          22:       35      // Fourth byte of the IP Address ({192,168,0,35})
        */        
        TPLinkDevices.clear();          // Clears list of existing devices. Gets repopulated from EEPROM
        uint8_t kasaDevCount = 0;       // Variable to hold the number of fetched devices from EEPROM
        uint8_t newDevIP[4];            // Variable to hold the IP address of the new device which is partially populated by the base IP
        for(uint8_t k = 0; k < 3; k++) newDevIP[k] = baseDevIP[k];  // Copy in the base IP from the microcontroller
        EEPROM.get(EEP_KASA_DEVS, kasaDevCount);        // Fetch the number of devices to be retrieved from EEPROM first
        //Serial.printlnf("Found %d devices stored.", kasaDevCount);
        for(uint16_t j = 0; j < kasaDevCount*2; j+=2){  // Loop for the number of devices stored in EEPROM, get their type and the fourth byte of their IP
            uint8_t devIP;              // Variable to be populated with the fourth byte of the IP
            uint8_t devType;            // Variable to be populated with the device type (0 = bulb, 1 = plug)
            EEPROM.get(EEP_KASA_DEVS + j + 1, devType); // Fetch the device type from the EEPROM
            EEPROM.get(EEP_KASA_DEVS + j + 2, devIP);   // Fetch the IP address from the EEPROM
            newDevIP[3] = devIP;        // Assign the fourth byte of the IP address to the value retrieved from memory
            if(devType == 0) addBulb(newDevIP);         // Based on the type, add a device to the dictionary
            else if(devType == 1) addPlug(newDevIP);
            //Serial.printlnf("Pulled device with IP: %d", newDevIP[3]);
        }
    }
    /// @brief Function to push the list of devices to EEPROM from the vector of TPLinkDevices
    void pushSavedDevices(){    //Take vector of TPLink Devices and overwrite stored memory
        /* EXAMPLE EEPROM
        Address   Value
          20:       3       // Number of stored devices (3)
          21:       0       // Device type of device 0 (bulb)
          22:       26      // Fourth byte of the IP Address ({192,168,0,26})
          21:       0       // Device type of device 0 (bulb)
          22:       28      // Fourth byte of the IP Address ({192,168,0,28})
          21:       1       // Device type of device 1 (plug)
          22:       35      // Fourth byte of the IP Address ({192,168,0,35})
        */
        uint8_t devIP;      // Variable to represent the fourth byte of this device's IP address
        uint8_t devType;    // Variable to represent the device type (0 = bulb, 1 = plug)
        uint8_t devCount = TPLinkDevices.size();    // Get the number of devices currently in the dictionary
        uint8_t iter = 1;   // Iterator for indexing through EEPROM locations
        EEPROM.put(EEP_KASA_DEVS,devCount);         // Store the number of devices in EEPROM for later fetching
        //Serial.printlnf("Stored %d devices.", devCount);
        for(TPLinkDevice dev: TPLinkDevices){       // Loop over all devices in the dictionary and store their config in the EEPROM
            devIP = dev.deviceIP[3];                // Get the fourth byte of the IP address from the dictionary
            devType = dev.deviceType;               // Get the device type from the dictionary (0 = bulb, 1 = plug)
            EEPROM.put(EEP_KASA_DEVS + (iter++), devType);  // Put the device type first
            EEPROM.put(EEP_KASA_DEVS + (iter++), devIP);    // Put the fourth byte of the IP address second
            //Serial.printlnf("Stored device with IP: %d", devIP);
        }
    }
};

/// @brief Class to do color conversion calculations based on brightness and Kelvin color temperature
class RGBColor{
public:
    int red, green, blue;   // Values to represent the RGB888 color. Ranges from 0-255
    float _brightness;      // Percentage of brightness (0-100)
    float _temperature;     // Kelvin color temperature (24-70)
    /// @brief Calculates the RGB values for a given kelvin temperature value
    /// @param kelvinTemp Kelvin color temperature (24-70)
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
    /// @brief Calculates the RGB values for a given kelvin temperature and brightness
    /// @param temperature Brightness percentage (0.0 - 1.0)
    /// @param brightness Kelvin color temperature (24 - 70), 24 = 2400K
    void convert_NB(float temperature, float brightness){
      _temperature = constrain(temperature, 0, 655);
      _brightness = constrain(brightness, 0.0, 1.0);
      float _green, _red, _blue;

      _red = _green = _blue = 0;
      float t = _temperature;
      
      // Do math based on what the color temperature is. Values below 6600K have different behavior
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

      // Cast intermediate value calculation
      red = (int)_red;
      green = (int)_green;
      blue = (int)_blue;
      
      // Ensure proper range of values (0-255)
      _normalize();
    }
    /// @brief Ensures the RGB values are within a valid range (0-255)
    void _normalize()
    {
      red   = constrain(_brightness * red,   0, 255);
      green = constrain(_brightness * green, 0, 255);
      blue  = constrain(_brightness * blue,  0, 255);
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

    // Extract the payload from the command string
    char payload[length];
    for(int i = 2; i < length; i++){
        payload[i-2] = input[i];
    }

    // Variables to be populated by the commands
    uint8_t targetBrightness = 0;
    uint8_t targetColor = 0;

    // Check that the commands were sent from the UI
    if(input[0] == 'U'){
        switch(input[1]){   // Second character in the command string is the command type
            case 'B': // Brightness command. Updated by the slider on the UI
                targetBrightness = atoi(payload);                   // Convert string to int
                if(targetBrightness > 100) targetBrightness = 100;  // Ensure values are within range
                currentBulbBrightness = targetBrightness / 100.0;   // Convert to 0.0 - 1.0 value
                updatedColorFromUI = true;                          // Set this here so the controller doesn't immediately echo back to the UI
                //Log.info("Received brightness command: %s. New brightness: %0.2f", payload, currentBulbBrightness);
                break;
            case 'C':   // Color temperature command. Updated by the slider on the UI
                targetColor = atoi(payload);                        // Convert string to int
                if(targetColor < 24) targetColor = 24;              // Ensure values are within range
                if(targetColor > 70) targetColor = 70;
                currentColorTemperature = targetColor;              // Assign range-mapped value to control variable
                updatedColorFromUI = true;                          //Set this here so the controller doesn't immediately echo back to the UI
                //Log.info("Received color temperature command: %s. New brightness: %0.2f", payload, currentColorTemperature);
                break;
            case 'G':   // Toggle command. Toggles light status
                autoControlLights = false;                          // Disable automatic light control when a user performs a manual action
                smartBulbState = !smartBulbState;                   // Invert the state of the bulbs
                //Log.info("Received toggle command: %s. New state: %d", payload, smartBulbState);
                break;
            case 'R':    // Request for all UI parameters
                sendParametersRequest = true;
                //Log.info("Received parameter request command: %s.", payload);
            default:
                break;
        }
    }
}

/// @brief Updates the state of the Kasa smart bulbs based on the global brightness and temperature variables
/// @param brightness Bulb brightness percentage (0 - 1.0)
/// @param color Bulb kelvin color temperature (24 - 70)
void updateBulbs(float brightness, float color){
    Serial.printlnf("Bulb Data Available: %d", DataAvailable());
    //Log.info("Updating Smart Bulbs. New brightness is %0.2f. New color temperature is %0.2f", brightness, color);
    //Bulbs are currently on (or turning on).
    if(smartBulbState){ 
        //Log.info("Smart bulbs are on. New brightness is %0.2f. New color temperature is %0.2f", brightness, color);
        TPLinkDeviceController.updateBulbColor(1, brightness, color);
        TPLinkDeviceController.setBulbs();
        currentColor.convert_NB(color, brightness);
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        delay(3);
    }
    // If the lights are on and the new state is off, ramp down the brightness slowly
    else if(lastSmartBulbState){
        //Log.info("Smart bulbs are turning off. New brightness is %0.2f. New color temperature is %0.2f", brightness, color);
        for(float i = brightness; i >= 0 ; i -= 0.05){
            TPLinkDeviceController.updateBulbColor(1, i, color);
            TPLinkDeviceController.setBulbs();
            currentColor.convert_NB(color, i);
            Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
            //delay(20);
        }
        TPLinkDeviceController.updateBulbColor(0, 0, color);
        TPLinkDeviceController.setBulbs();
        //Log.info("Smart bulbs have been turned off.");
        lastSmartBulbState = smartBulbState;
    }
    //Bulbs done ramping down. Set them to off.
    else{
        //Log.info("Smart bulbs are off");
        TPLinkDeviceController.updateBulbColor(0,0,color);
        TPLinkDeviceController.setBulbs();
        Serial1.printlnf("MF%x %x %x", 0, 0, 0);
    }
    
}

/// @brief Takes readings of the ambient light sensor and calculates an average brightness
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

    lcdBacklightPercentage = map(average, 0, max_brightness, 1, 100);
    if(lcdBacklightPercentage >= 100) lcdBacklightPercentage = 100;

    // Check if the average has changed by more than 3%
    if (abs(lcdBacklightPercentage - lastBrightness) >= 3) {
        // Transmit the new average over Serial1
        //Log.info("Backlight brightness has changed to %d", lcdBacklightPercentage);
        Serial1.printlnf("ML%d", lcdBacklightPercentage);
        lastBrightness = lcdBacklightPercentage; // Update the last average
    }
}

/// @brief Updates the states of some elements on the frontend by sending commands
void updateUI(){
    static int lastClockMinute = -1;
    static uint32_t lastBulbBorderColor = 0;

    // Check if the phone has been recently detected
    if(phonePresenceIcon != (System.millis() - discoveredPhoneTimestamp < BT_TIMEOUT_MS)){
        phonePresenceIcon = !phonePresenceIcon;
        Serial1.printlnf(phonePresenceIcon ? "MP1" : "MP0");
        //Log.info("Phone state has changed. New value: %d", phonePresenceIcon);
    }

    // Check if the watch has been recently detected
    if(watchPresenceIcon != (System.millis() - discoveredWatchTimestamp < BT_TIMEOUT_MS)){
        watchPresenceIcon = !watchPresenceIcon;
        Serial1.printlnf(watchPresenceIcon ? "MW1" : "MW0");
        //Log.info("Watch state has changed. New value: %d", watchPresenceIcon);
    }

    // Check if the time has changed
    if(lastClockMinute != Time.minute()){
        lastClockMinute = Time.minute();
        Serial1.printlnf("MT%d %d", Time.hourFormat12(), Time.minute());
        //Log.info("Clock value has changed. New value: %d:%d", Time.hourFormat12(), Time.minute());
    }

    // Check if we are in a smart control mode. Change the border color accordingly
    uint32_t bulbBorderColor = 0xFFFFFF;                    //White
    if(autoBrightnessActive) bulbBorderColor = 0x00FF00;    //Green
    else if(autoColorRamping) bulbBorderColor = 0x7B3DFF;   //Light purple
    if(bulbBorderColor != lastBulbBorderColor){
        Serial1.printlnf("MO%x %x %x", (unsigned int)((bulbBorderColor >> 16) & 0xFF), (unsigned int)((bulbBorderColor >> 8) & 0xFF), (unsigned int)(bulbBorderColor & 0xFF));
        lastBulbBorderColor = bulbBorderColor;
        //Log.info("UI bulb outline color has changed to 0x%X", bulbBorderColor);
    }

}

/// @brief Sends all parameters for the frontend to have it be updated upon restart
void sendAllParameters(){
    // Check if frontend UI has requested the parameters to be sent
    if(sendParametersRequest){
        currentColor.convert_NB(currentColorTemperature, currentBulbBrightness);
        
        //Log.info("All parameters were requested by the UI");
        //Log.info("Phone state is: %d", phonePresenceIcon);
        //Log.info("Watch state is: %d", watchPresenceIcon);
        //Log.info("LCD backlight percentage is: %d", lcdBacklightPercentage);
        //Log.info("Current bulb brightness: %d", (int)(currentBulbBrightness*100));
        //Log.info("Current color temperature: %d", (int)(currentBulbBrightness*100));
        //Log.info("Current time: %d:%d", Time.hourFormat12(), Time.minute());
        //Log.info("Current bulb fill color: %x %x %x", currentColor.red, currentColor.green, currentColor.blue);

        Serial1.printlnf("MP%d", phonePresenceIcon ? 1 : 0);
        Serial1.printlnf("MW%d", watchPresenceIcon ? 1 : 0);
        Serial1.printlnf("ML%d", lcdBacklightPercentage);
        Serial1.printlnf("MB%d", (int)(currentBulbBrightness*100));
        Serial1.printlnf("MC%d", (int)currentColorTemperature);
        Serial1.printlnf("MT%d %d", Time.hourFormat12(), Time.minute());
        Serial1.printlnf("MF%x %x %x", currentColor.red, currentColor.green, currentColor.blue);
        sendParametersRequest = false;
    }
    
}

/// @brief Checks if any serial commands are available from the frontend and processes them
void checkForSerialCommands(){
    if(Serial1.available()){
        char input[32];
        int length = Serial1.readBytesUntil('\n', input, sizeof(input));
        if(length > 0){
            //Log.info("Received serial command: %s", input);
            processCommand(input, length);
        }
    }
}

/// @brief Function run once at startup
void setup() {

    //Log.info("Beginning serial communication");
    Serial.begin(115200);
    Serial1.begin(115200);
    WiFi.selectAntenna(ANT_INTERNAL);
    
    //Log.info("Enabling Bluetooth LE");
    BLE.on();
    //BLE.selectAntenna(BLE_ANT_EXTERNAL);
    BLE.setScanTimeout(50);

    //Log.info("Registering Particle functions with the cloud");
    Particle.function("lightControl", webhookCommandHandler);
    Particle.function("SetLightColor", setBulbColor);
    Particle.variable("Light Sensor", lcdBacklightPercentage);
    //Particle.variable("Color Temperature", currentColorTemperature);
    
    //Log.info("Configuring GPIO pins");
    pinMode(BRTSNS, INPUT);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    //Log.info("Overriding EEPROM registries");
    //EEPROM.write(EEP_MODE_REG,(uint8_t)1);
    EEPROM.write(EEP_BRT_REG,(uint8_t)100);
    EEPROM.write(EEP_COL_REG,(uint8_t)25);

    //for(int k = 0; k < (BT_MAXDEVICES>>1); k++) EEPROM.write(k,0);
    //EEPROM.write(EEP_DEVCOUNT_REG,2);

    //EEPROM.write(0,7);
    //EEPROM.write(1,31);
    //EEPROM.write(2,5);
    //EEPROM.write(3,152);

    //Log.info("Fetching saved BLE devices from EEPROM");
    EEPROM.get(EEP_DEVCOUNT_REG, storedBLEDeviceCount);          //Populate device count from EEPROM
    for(int k = 0; k < (storedBLEDeviceCount << 1); k++){     //Populate data keys from EEPROM
        EEPROM.get(k,storedBLEKeys[k]);

    }
    for(int k = 0; k < storedBLEDeviceCount; k++){          //Set device RSSI's
        devRSSI[k] = 65;
    }
    devRSSI[1] = 100;
    doBLERangeScanning = true;

    lastBLEDetectionTime = 0;
    lastBLEScanTime = System.millis();
    autoControlLights = true;
    reEnableAutoControl = false;

    //Log.info("Attempting to connect to Particle cloud");
    Particle.connect();
    while(WiFi.connecting()){

        //delay(5);
    }
    //Log.info("Wi-Fi has finished connecting. Attempting connection to Particle cloud");
    while(!Particle.connected()){

    }
    //Log.info("Particle cloud has been connected");
    //WiFi.localIP().toString().c_str()
    delay(1000);

    //Log.info("Fetching last brightness and color temperature from EEPROM");
    currentColorTemperature = EEPROM.read(EEP_COL_REG);
    currentBulbBrightness = EEPROM.read(EEP_BRT_REG);
    currentBulbBrightness /= 100.0;
    lastBulbBrightness = 0.0;
    if(currentBulbBrightness > 1) currentBulbBrightness = 1;
    if(currentColorTemperature < 24) currentColorTemperature = 24;
    if(currentColorTemperature > 70) currentColorTemperature = 70;
    //Log.info("Fetched values %0.2f for brightness and %0.2f for color temperature", currentBulbBrightness, currentColorTemperature);

    //currentBulbBrightness = EEPROM.read(EEP_BRT_REG) / 100.0;
    //currentColorTemperature = EEPROM.read(EEP_COL_REG);
    
    RGB.control(true);
    RGB.color(0,0,0);

    Time.zone(-4);

    //FOR /L %i IN (1,1,254) DO @ping -n 1 -w 200 192.168.1.%i | FIND /i "TTL"

    //Log.info("Adding bulbs to TP-Link Controller");
    //TPLinkDeviceController.addBulb(bulbIP1);
    //TPLinkDeviceController.addBulb(bulbIP2);
    //TPLinkDeviceController.addBulb(bulbIP3);
    //TPLinkDeviceController.pushSavedDevices();

    //Log.info("Fetching bulbs from EEPROM and adding to TP-Link Controller");
    IPAddress localIP = WiFi.localIP();
    uint8_t ipBytes[4] = {localIP[0], localIP[1], localIP[2], localIP[3]};
    TPLinkDeviceController.pullSavedDevices(ipBytes);
      
    /*HMD.begin();
    delay(10);
    HMD.Mode(0);
    HMD.MotorSelect(0x63); //0x63
    HMD.Library(7); //change to 6 for LRA motors 
    updateSelectedItems();
    HMD.Waveform(0, 42); //56
    HMD.go();*/

    //Log.info("Setup function has completed");


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
    checkForSerialCommands();
    
    // If the lights are off, and automatic light control is enabled, check if a bluetooth device has been recently discovered. Only turn on lights if room brightness is dark.
    if(smartBulbState == false && autoControlLights){      
        if(System.millis()-lastBLEDetectionTime < BT_TIMEOUT_MS && analogRead(BRTSNS) < BT_BRT_AUTO_ON){   //Check that the brightness sensor is dark so doesn't turn on during the day
            //Log.info("A device has been detected. Automatically turning on smart bulbs.");
            autoColorRamping = true;
            smartBulbState = true;
        }
    }
    //If the lights are on and automatic control is enabled, check if a bluetooth device has been recently discovered. Turn off lights if it has been too long.
    else if(autoControlLights){ // If lamp is on, turn of iff timed out
        if(System.millis() - lastBLEDetectionTime > BT_TIMEOUT_MS){
            //Log.info("No devices have been detected recently. Automatically turning off smart bulbs.");
            smartBulbState = false;
            autoBrightnessActive = false;
        }
    }
    // If the lights are off and automatic control is disabled, check how long a device has been away for. If a device has been away for so long, re-enable automatic control
    // If devices are still in the room, it will keep automatic control disabled until the device leaves for an extended period of time and then comes back.
    else if(!reEnableAutoControl){
        if(System.millis()-lastBLEDetectionTime > BT_AUTO_RE_EN_MS){  //Re-enable scanner if device has disappeared for a longer period of time
            //Log.info("Automatic light control has been re-enabled, looking for devices...");
            reEnableAutoControl = true;
        }
    }
        
        

    ///////////////////////////////////////////////////
    //  Automatic Brightness and Temperature Control //
    ///////////////////////////////////////////////////
    if(autoBrightnessActive && System.millis() - autoOnTimestamp > 30000 && smartBulbState){
        if(fadeUpMinutesElapsed < FADETIME_MIN*2){
            if(currentBulbBrightness < 1){
                currentBulbBrightness += (double)((1.0-ONBRT)/(FADETIME_MIN*2));
                if(currentBulbBrightness > 1) currentBulbBrightness = 1;
                //Log.info("Bulb brightness has been adjusted by ramp-up feature. New brightness: %0.2f", currentBulbBrightness);
            }
            //if(currentColorTemperature > targetColorTemperature){   //Temperature ramp-up on auto-on
            //    currentColorTemperature  -= (double)((ONTEMP-targetColorTemperature)/(FADETIME_MIN*2.0));
            //    if(currentColorTemperature < targetColorTemperature) currentColorTemperature = targetColorTemperature;
            //}
        }
        else{
            autoBrightnessActive = false;
            autoColorRamping = true;
            targetColorTemperature = ENDTEMP;
            //Log.info("Automatic bulb brightness has reached target maximum. Returning to fixed brightness %0.2f.", currentBulbBrightness);
        }
        fadeUpMinutesElapsed++;
        autoOnTimestamp = System.millis();
    }
        
    if(autoColorRamping && HRENDFADE > HRSTARTFADE && Time.hour() >= HRSTARTFADE && smartBulbState){// && Time.hour() < HRENDFADE){  //Not fading up in brightness. Automatically warm color temperature over time.
        if(Time.hour() < HRENDFADE && System.millis() - autoOnTimestamp > 30000){    //While in the ramp-down period
            if(autoColorRamping && currentColorTemperature > ENDTEMP){
                float diff = ((((double)(HRENDFADE - Time.hour()) - ((double)Time.minute()/60.0)) / (double)(HRENDFADE - HRSTARTFADE)) * ((double)EEPROM.read(EEP_COL_REG) - ENDTEMP));
                if(diff < 0) diff = 0;
                currentColorTemperature = diff + ENDTEMP;   //Linear temperature based on time between start and end of fading period
                //Log.info("Color temperature has been adjusted by ramp-down feature. New color temperature: %0.2f. Ramp-down percentage: %0.2f", currentColorTemperature, ((double)(HRENDFADE - Time.hour()) - ((double)Time.minute()/60.0)));
            }
            else{   //Reached end of ramp-down period
                autoColorRamping = false;
                //Log.info("Ramp-down period for color temperature has ended. Final color temperature: %0.2f", currentColorTemperature);
            }
            autoOnTimestamp = System.millis();
        }
    }

    //if(Time.hour() == HRRESETCOLOR && autoColorRamping) currentColorTemperature = EEPROM.read(EEP_COL_REG);

    // Scan for bluetooth devices to detect proximity. Populates lastBLEDetectionTime if devices are found.
    if(System.millis() - lastBLEScanTime > BLE_SCAN_PD && doBLERangeScanning){
        lastBLEScanTime = System.millis();
        //Log.info("Initiating scan for BLE devices");
        BLEScan();
    }

    //Check if an update has been made to brightness, color, or the state of the bulbs and sends the TCP commands if there are updates.
    if(updateBrightness() || updateColor() || smartBulbState != lastSmartBulbState){
        //Log.info("Bulb parameters have changed. Applying updates to bulbs.");
        updateBulbs(currentBulbBrightness, currentColorTemperature);
        lastSmartBulbState = smartBulbState;
    }
    
    delay(10);
}

/// @brief Function to check if the brightness value has changed and push an update to the frontend
/// @return Flag indicating if the brightness was updated
bool updateBrightness(){
    if(currentBulbBrightness != lastBulbBrightness){
        lastBulbBrightness = currentBulbBrightness;
        //EEPROM.write(EEP_COL_REG,(uint8_t)(currentBulbBrightness*100));
        currentColor.convert_NB(currentColorTemperature, currentBulbBrightness);
        if(!updatedColorFromUI) Serial1.printlnf("MB%d", (int)(currentBulbBrightness*100));
        updatedColorFromUI = false;
        return true;
    }
    return false;
}

/// @brief Function to check if the color temperature value has changed and push an update to the frontend
/// @return Flag indicating if the color temperature was updated
bool updateColor(){
    if(currentColorTemperature != lastColorTemperature){
        lastColorTemperature = currentColorTemperature;
        //EEPROM.write(EEP_BRT_REG,currentColorTemperature);
        currentColor.convert_NB(currentColorTemperature, currentBulbBrightness);
        if(!updatedColorFromUI) Serial1.printlnf("MC%d", (int)currentColorTemperature);
        updatedColorFromUI = false;
        return true;
    }
    return false;
}

/// @brief Function to handle function calls from the Particle API. Used with Siri Shortcuts to allow for phone control
/// @param command String command parameter with the command
/// @return Success or failure value
int webhookCommandHandler(const char *command)
{
    // Command to toggle on/off smart bulbs while maintaining brightness/color temperature
    if(strcmp(command,"toggle")==0){   
        autoControlLights = false;
        autoControlLights = false;
        smartBulbState = !smartBulbState;
        //Log.info("Toggle command was received from the cloud. New bulb state: %d", smartBulbState); 
    }
    // Command to toggle off smart bulbs while maintaining brightness/color temperature
    else if(strcmp(command,"evoff")==0){
        autoControlLights = false;
        smartBulbState = false;
        autoBrightnessActive = false;
        //Log.info("Lights-off command was received from the cloud. New bulb state: %d", smartBulbState); 
    }
    // Command to toggle on smart bulbs while maintaining brightness/color temperature
    else if(strcmp(command,"evon")==0){
        autoControlLights = false;
        smartBulbState = true;
        autoBrightnessActive = false;
        //Log.info("Lights-on command was received from the cloud. New bulb state: %d", smartBulbState); 
    }
    // Command to set mode to automatic brightness ramping. Gets called near sunset using Siri Shortcuts.
    else if(strcmp(command,"autoon")==0){
        autoBrightnessActive = true;
        fadeUpMinutesElapsed = 0;
        fadeUpStartHour = Time.hour()+1;
        autoOnTimestamp = System.millis() + 30000;
        targetColorTemperature = ONTEMP;
        currentBulbBrightness = ONBRT;
        currentColorTemperature = EEPROM.read(EEP_COL_REG);
        //Log.info("Automatic-on command was received from the cloud. Enabling automatic brightness fade-up");
    }
    // Command to set mode to automatic color ramping. Warms temperature as time gets later.
    else if(strcmp(command,"autocolor")==0){
        if(autoColorRamping){
            autoColorRamping = false;
            currentColorTemperature = EEPROM.read(EEP_COL_REG);
            //Log.info("Automatic color temperature command received from the cloud. Toggling off.");
        } 
        else{
            autoColorRamping = true;
            //Log.info("Automatic color temperature command received from the cloud. Toggling on.");
        }
    }
    // Turns on light control via bluetooth proximity scanning
    else if(strcmp(command,"bton")==0){
        doBLERangeScanning = true;
    }
    // Turns off light control via bluetooth proximity scanning
    else if(strcmp(command,"btoff")==0){
        doBLERangeScanning = false;
    }
    // Turns on smart plug #1
    else if(strcmp(command,"plg1on")==0){
        TPLink_Plug(plugIP1, 1);
    }
    // Turns on smart plug #2
    else if(strcmp(command,"plg2on")==0){
        TPLink_Plug(plugIP2, 1);
    }
    // Turns on smart plug #3
    else if(strcmp(command,"plg3on")==0){
        TPLink_Plug(plugIP3, 1);
    }
    // Turns on smart plug #3
    else if(strcmp(command,"plg4on")==0){
        TPLink_Plug(plugIP4, 1);
    }
    // Turns off smart plug #1
    else if(strcmp(command,"plg1off")==0){
        TPLink_Plug(plugIP1, 0);
    }
    // Turns off smart plug #2
    else if(strcmp(command,"plg2off")==0){
        TPLink_Plug(plugIP2, 0);
    }
    // Turns off smart plug #3
    else if(strcmp(command,"plg3off")==0){
        TPLink_Plug(plugIP3, 0);
    }
    // Turns off smart plug #4
    else if(strcmp(command,"plg4off")==0){
        TPLink_Plug(plugIP4, 0);
    }
    // Sets brightness of bulbs to 25%
    else if(strcmp(command,"pct25")==0){
        currentBulbBrightness = 0.25;
        smartBulbState = true;
    }
    // Sets brightness of bulbs to 50%
    else if(strcmp(command,"pct50")==0){
        currentBulbBrightness = 0.50;
        smartBulbState = true;
    }
    // Sets brightness of bulbs to 75%
    else if(strcmp(command,"pct75")==0){
        currentBulbBrightness = 0.75;
        smartBulbState = true;
    }
    // Sets brightness of bulbs to 100%
    else if(strcmp(command,"pct100")==0){
        currentBulbBrightness = 1;
        smartBulbState = true;
    }
    return 1;
}

/// @brief Scans for BLE devices and determines if they are in range of the controller.
void BLEScan(){        //Function to scan for nearby bots advertising over BLE or connect to a specific one.
    size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);                                //Get number of found BLE devices
	if (count > 0) {                                                                        //Check if any were found
		for (uint8_t ii = 0; ii < count; ii++) {                                            //Loop over discovered bots
			//if(scanResults[ii].rssi() < 0 && scanResults[ii].rssi() > -45){
                uint8_t bufName[32];                                           //Array to hold the bytes of custom data
                scanResults[ii].advertisingData().customData(bufName,CUSTOM_DATA_LEN);      //Retrieve the custom data from the advertisement
                //size_t cDataLen = scanResults[ii].advertisingData().customData(bufName,CUSTOM_DATA_LEN);      //Retrieve the custom data from the advertisement
                //if(cDataLen < 8 || cDataLen > 12) continue;
                //Serial.printlnf("Custom data len: %d",cDataLen);
                //uint8_t advData[32];                                                        //Buffer that gets populated with BLE advertising data
                //size_t aDataLen = scanResults[ii].advertisingData().get(advData,CUSTOM_DATA_LEN);                          //Populate the buffer
                //Serial.printlnf("RSSI: %d    DATA: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",scanResults[ii].rssi(), advData[0], advData[1],advData[2],advData[3],advData[4], advData[5],advData[6],advData[7], bufName[0], bufName[1], bufName[2], bufName[3], bufName[4], bufName[5], bufName[6], bufName[7], bufName[8], bufName[9], bufName[10],bufName[11],bufName[12],bufName[13], bufName[14], bufName[15], bufName[16], bufName[17], bufName[18], bufName[19], bufName[20], bufName[21], bufName[22], bufName[23], bufName[24],bufName[25],bufName[26],bufName[27]);
                //if(bufName[0] == 76) Serial.printlnf("Found an iDevice %d %d", bufName[3], bufName[5]);
                uint8_t deviceNumber;                                                       //Temporary variable to hold the device numbers based on the EEPROM retrieved value
                for(deviceNumber = 0; deviceNumber < storedBLEDeviceCount; deviceNumber++){     //Loop over the devices saved in memory
                    if(bufName[0] == 76 && bufName[3] == storedBLEKeys[deviceNumber << 1] && bufName[5] == storedBLEKeys[(deviceNumber << 1)+1] && scanResults[ii].rssi() > (0-devRSSI[deviceNumber])){   //Check if this device matches any of the saved devices
                        //Serial.printlnf("Found Device IDs: %d %d, RSSI: %d", storedBLEKeys[deviceNumber << 1], storedBLEKeys[(deviceNumber << 1)+1], scanResults[ii].rssi());                             
                        lastBLEDetectionTime = System.millis();                                        //Save the time that a saved device was last seen, if too much time elapses since this time was saved, then turn off the lights
                        if(devRSSI[deviceNumber] < 70) discoveredPhoneTimestamp = System.millis();   //Strong RSSI devices are phones, update the phone update time for updating the phone icon
                        else discoveredWatchTimestamp = System.millis();                             //Strong RSSI devices are watches, update the phone update time for updating the watch icon
                        if(reEnableAutoControl){
                            autoControlLights = true;
                            reEnableAutoControl = false;
                            //Serial.printlnf("Found device, re-enabling automatic control");
                       }
                    }
                }
            //}
        }
	}
}

/// @brief Function to allow changing of bulb colors via a Particle command
/// @param command String command parameter from the Particle function call
/// @return Value indicating success or failure
int setBulbColor(const char *command){
    int receivedVal;
    receivedVal = atoi(command);
    int RedVal = receivedVal%1000;
    int GreenVal = (receivedVal/1000)%1000;
    int BlueVal = (receivedVal/1000000)%1000;
    TPLink_RGB_Bulb(bulbIP2, 1, RedVal, GreenVal, BlueVal);
    return 1;
}

/// @brief Stores EEPROM values for the brightness and color temperatures
void updateEEPROM(){
    EEPROM.write(EEP_BRT_REG,(uint8_t)(currentBulbBrightness*100.0));
    EEPROM.write(EEP_COL_REG,(uint8_t)currentColorTemperature);
    //Log.info("Updating EEPROM values for brightness to %0.2f and color temperature to %0.2f", currentBulbBrightness, currentColorTemperature);
    eepromUpdate.stopFromISR();
}
