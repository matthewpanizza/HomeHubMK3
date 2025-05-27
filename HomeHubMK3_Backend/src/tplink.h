/*
 * Project TPLink
 * Description: Routines to Communicate to TPLink products
 * Author: Martin Smaha
 * Date:3/27/17
 */
 void TPLink_Bulb(uint8_t * lampIP, int state, int percent, int coltemp);
 void TPLink_LB130Bulb(uint8_t * lampIP, int state, int hue, int saturation, int percent);
 void TPLink_RGB_Bulb(uint8_t * lampIP, int state, int Red, int Green, int Blue); //LB130 alternate call
 void TPLink_Plug(uint8_t * plugIP, int state);
 void rgbToHsl(int Red, int Green, int Blue, int * HSL);//converion routine
 int DataAvailable();
