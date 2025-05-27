/*
 * Project TPLink
 * Description: Routines to Communicate to TPLink products
 * Author: Martin Smaha
 * Date:3/27/17
 */
#include "Particle.h"
#include "tplink.h"

 unsigned int localPort = 8888; //local port to use
 unsigned int TPLinkPort = 9999;//TPLink protocol port
 UDP Udp;
 

// LB100 / LB110 Bulb control parameters are
//{"smartlife.iot.smartbulb.lightingservice":{"transition_light_state":{"on_off":1,"brightness":70,"mode":"normal","ignore_default":1,"color_temp":2700,"transition_period":150}}}
// HS100 / HS110 / HS105 Plug control parameters are
//{system":{"set_relay_state":{"state":1}}}
// LB130 Bulb control parameters are
//{"smartlife.iot.smartbulb.lightingservice":{"transition_light_state":{"on_off":1,"mode":"normal","hue":120,"saturation":65,"color_temp":0,"brightness":100,"err_code":0}}}

// protocol encoder
 int encode(char * xData) { //encodes JSON text string passed in parameter
   int length = strlen(xData);
   char key = 171;
   for (int j = 0; j < length; j++) {
     char b = (key ^ xData[j]);
     key = b;
     xData[j] = b;
   }
   return length;
 }

int DataAvailable(){
  return Udp.available();
}

// encode and send a UDP packet to the target TPLink device
void SendPacket(uint8_t * targetIP, char * xPacket) {
  IPAddress IPfromBytes;
  IPfromBytes = targetIP;
  int len = encode(xPacket);
  Udp.begin(localPort);
  Udp.sendPacket(xPacket, len, IPfromBytes, TPLinkPort);
  Udp.stop();
}
// light bulb control packet
// NOTE: you can only change brightness while bulb is ON
void TPLink_Bulb(uint8_t * lampIP, int state, int percent, int colTemp) {
  char buf[512];
  if (percent == 0) {
    state = 0;//force off
  }
  sprintf(buf,"{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":{\"on_off\":%d,\"brightness\":%d,\"mode\":\"normal\",\"ignore_default\":1,\"color_temp\":%d,\"transition_period\":0}}}",state,percent, colTemp);
  SendPacket(lampIP, buf);
}
// color bulb control packet
void TPLink_LB130Bulb(uint8_t * lampIP, int state, int hue, int saturation, int percent) {
  char buf[512];
  sprintf(buf,"{\"smartlife.iot.smartbulb.lightingservice\":{\"transition_light_state\":{\"on_off\":%d,\"hue\":%d,\"saturation\":%d,\"brightness\":%d,\"mode\":\"normal\",\"ignore_default\":1,\"color_temp\":0,\"transition_period\":0}}}",state,hue,saturation,percent);
  SendPacket(lampIP, buf);
}
//alternate call
void TPLink_RGB_Bulb(uint8_t * lampIP, int state, int Red, int Green, int Blue) {
  if ((Red == 0 && Green == 0 && Blue == 0) || state == 0) {
    TPLink_LB130Bulb(lampIP,0,0,0,0);
  } else {
    int hsl[3];
    rgbToHsl(Red, Green, Blue, hsl);
    TPLink_LB130Bulb(lampIP,state,hsl[0],hsl[1],hsl[2]);
  }
}
// plug control packet
void TPLink_Plug(uint8_t * plugIP, int state) {
  char buf[512];
  sprintf(buf,"{\"system\":{\"set_relay_state\":{\"state\":%d}}}",state);
  SendPacket(plugIP, buf);
}
// RGB to HSL conversion
/**
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes Red, Green, and Blue contained in the set [0, 255] and
 * returns h, s, and l as integers ready for TPLink
 *
 */

void rgbToHsl(int Red, int Green, int Blue, int * HSL) {
    float r = float(Red) / 255.0f;
    float g = float(Green) / 255.0f;
    float b = float(Blue) / 255.0f;

    float max = (r > g && r > b) ? r : (g > b) ? g : b;
    float min = (r < g && r < b) ? r : (g < b) ? g : b;

    float h, s, l;
    l = (max + min) / 2.0f;

    if (max == min) {
        h = s = 0.0f;
    } else {
        float d = max - min;
        s = (l > 0.5f) ? d / (2.0f - max - min) : d / (max + min);

        if (r > g && r > b)
            h = (g - b) / d + (g < b ? 6.0f : 0.0f);

        else if (g > b)
            h = (b - r) / d + 2.0f;

        else
            h = (r - g) / d + 4.0f;

        h /= 6.0f;
    }
    h *= 360;
    s *= 100;
    l *= 100;
    HSL[0] = h;
    HSL[1] = s;
    HSL[2] = l;
}
