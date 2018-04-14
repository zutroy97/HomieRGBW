#define FW_NAME "RGBWController"
#define FW_VERSION "0.0.4"

#include <RGBConverter.h>
#include <ArduinoJson.h>

#include <Homie.h>
#include <RGBWWLed.h>

// LED related
#define BLUEPIN 14
#define GREENPIN 12
#define REDPIN 13
#define WWPIN 5
#define CWPIN 2

#define TRANSITION_DEFAULT 1000

// RF Remote
// If a pulse is between these two values, consider it a Header pulse
#define MIN_HEADER_LENGTH 4000
#define MAX_HEADER_LENGTH 8000

// between these two values, cosider it a 0
#define MIN_SPACE_LENGTH 150
#define MAX_SPACE_LENGTH 250

// between these two values, consider it a 1
#define MIN_MARK_LENGTH 500
#define MAX_MARK_LENGTH 750

// Command values for each button
#define BUTTON_POWER_ON     0x01
#define BUTTON_POWER_OFF    0x02
#define BUTTON_UP           0x03
#define BUTTON_DOUBLE_ARROW 0x06
#define BUTTON_MINUS        0x04
#define BUTTON_PLUS         0x08
#define BUTTON_DOWN         0x07
#define BUTTON_RED          0x05
#define BUTTON_GREEN        0x0C
#define BUTTON_BLUE         0x0B
#define BUTTON_WHITE        0x09

// ID of the remote to respond to
#define REMOTE_ID           0x2686
#define REMOTE_REPEAT_TIME  (100 * 1000) // 100 ms

#define REMOTE_RF_PIN       4 // Input from RF module output

// Contains data about the RF packet
typedef union RFPacket {
  struct{
    byte command :8;
    unsigned int remote :16;
  } id;
  unsigned long value;
} ;

// Used when receiving/building a packet
volatile struct RemoteCommand
{
  byte count;
  union RFPacket packet;
  byte isReady = 0;
  unsigned long receiveTime;
} receivedCommand;

HomieNode lightNode("light", "color");
RGBWWLed rgbled;
RGBConverter colorConverter;

HSVCT colorBlack = HSVCT(0, 0, 0); 
HSVCT colorWhite = HSVCT(0.0f, 0.0f, 100.0f); 
HSVCT colorRed = HSVCT(0.0f, 100.0f, 100.0f);  
HSVCT colorBlue = HSVCT(240.0f, 100.0f, 100.0f); 
HSVCT colorGreen = HSVCT(120.0f, 100.0f, 100.0f); 

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */


void ICACHE_RAM_ATTR handleRfInterrupt()
{
  static union RFPacket workingPacket;
  static unsigned long lastChange = 0;
  unsigned long now = micros();
  static bool isHighBitPosition = false;
  static bool isCapturing = false;
  static byte bitPosition = 0;
  bool value = 0;
  unsigned long duration = now - lastChange;
  lastChange = now; // Remember for next time around
  //Serial.print(" "); Serial.print(duration);

  // Do we have a Header pulse (Long duration. We know a duration this long is HIGH)
  if (duration >= MIN_HEADER_LENGTH && duration <= MAX_HEADER_LENGTH)
  {
    //Serial.println(bitPosition);Serial.print("H");
    workingPacket.value = 0; // Reset our values
    isCapturing = true; // Start capturing
    isHighBitPosition = false; // Ignore the next (low) pulse
    bitPosition = 0; // Start building the value from 0
    return;
  }

  if (isCapturing == false)
  {
    return; // If we aren't capturing, nothing to do
  }
  if (duration >= MIN_SPACE_LENGTH && duration <= MAX_SPACE_LENGTH)
  {
    value = 0; // Found a small pulse (high or low, doesn't matter)
  }else if (duration >= MIN_MARK_LENGTH && duration <= MAX_MARK_LENGTH){
    value = 1; // Found a long pulse (has to be high,
  }else{
    isCapturing = 0; // Our packet encountered interferrence. Abort.
    //Serial.print("X");
    return;
  }
  if (isHighBitPosition)
  { // Are we be interested in this pulse?
    bitPosition++; // Keep track of how many bits we've collected.
    workingPacket.value = (workingPacket.value << 1) | value; // Shift left and append value to least significant bit

    if (bitPosition == 24) // Do we have the entire packet?
    {
      //Serial.print("RID: 0x"); Serial.print(workingPacket.id.remote, HEX);
      //Serial.print(" CID: 0x"); Serial.println(workingPacket.id.command, HEX);
      //Serial.print(" Value: "); Serial.println(workingPacket.value);
      isCapturing = 0; // Don't capture until next header
      if (receivedCommand.isReady == 0)
      {
        if ((receivedCommand.packet.value != workingPacket.value) 
          || ( (now - receivedCommand.receiveTime) > REMOTE_REPEAT_TIME ) )
        { // Different Packet as last time OR timeout occurred
          receivedCommand.packet.value = workingPacket.value;
          receivedCommand.count = 0;
        }else{
          receivedCommand.count++;
        }
        receivedCommand.receiveTime = now;    
        receivedCommand.isReady = 1;
        //Serial.println("R");
      }
    }
  }
  isHighBitPosition = !isHighBitPosition;
}

bool hassSetHandler(String value)
{
  byte rgbw[5] = {0, 0, 0,0,0};
  double hsv[3] = { 0.0, 0.0, 0.0 };
  HSVCT color;
  int transition = TRANSITION_DEFAULT;
  
  StaticJsonBuffer<350> jsonBuffer;
  //delay(1); //short break for esp to catch up
  JsonObject& root = jsonBuffer.parseObject(value);
  //delay(1); //short break for esp to catch up
  if (!root.success()){
    Serial.println("Bad JSON");
    return false;
  }

  if (root.containsKey("transition"))
  {
    transition = root["transition"];
  }
  if (root.containsKey("brightness"))
  {
    int brightness = root["brightness"];
    color = rgbled.getCurrentColor();
    color.v = map(brightness, 0, 255, 0, RGBWW_CALC_MAXVAL);
    setColor(color, transition);
    return true;
  }
  if (root.containsKey("state"))
  {
    if (root["state"] == "OFF")
    {
      setColor(colorBlack, transition);
      return true;
    }else if (root["state"] == "ON" && !isLightOn()){
      setColor(colorWhite, transition);
      return true;
    }
  }

  if (root.containsKey("color"))
  {
    JsonObject& oColor = root["color"];
    rgbw[0] = oColor["r"];
    rgbw[1] = oColor["g"];
    rgbw[2] = oColor["b"];
    colorConverter.rgbToHsv(rgbw[0], rgbw[1], rgbw[2], hsv);
    color = HSVCT( (float)(hsv[0] * 360.0), (float)(hsv[1] * 100.0), (float)(hsv[2] * 100.0) );
    setColor(color, transition);
    return true;
  }
  Serial.print("Unknown Command: "); Serial.println(value);
  return false;
}

void onColorChanged(RGBWWLed* rgbwwctrl)
{
  char buffer[256];
  StaticJsonBuffer<400> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  //delay(1); //short break for esp to catch up
  JsonObject& color = jsonBuffer.createObject();
  HSVCT c = rgbwwctrl->getCurrentColor();
  root["brightness"] = map(c.v, 0, RGBWW_CALC_MAXVAL, 0, 255);
  SetAsRadian(c);
  byte rgb[3];
  colorConverter.hsvToRgb( c.h / 360.0, c.s / 100.0, c.v / 100.0, rgb);
  //delay(1); //short break for esp to catch up
  color["r"] = rgb[0];
  color["g"] = rgb[1];
  color["b"] = rgb[2];
  root["color"] = color;
  root["state"] = isLightOn() ? "ON" : "OFF";

  root.printTo(buffer, sizeof(buffer));
  Homie.setNodeProperty(lightNode, "hass", buffer, false);
}

void doStartUpColors()
{
  addToFadeQueue(colorRed, 500);  addToFadeQueue(colorBlack, 500); 
  addToFadeQueue(colorWhite, 500);  addToFadeQueue(colorBlack, 500); 
  addToFadeQueue(colorBlue, 500);  addToFadeQueue(colorBlack, 500); 
}

void adjustBrightness(int value)
{
  HSVCT currentColor = rgbled.getCurrentColor();
  int oldValue = currentColor.value;
  currentColor.value += value;
  currentColor.value = constrain(currentColor.value, 0, RGBWW_CALC_MAXVAL);
  if (oldValue != currentColor.value){
    setColor(currentColor, 0);
  }
}

void adjustHue(int value)
{
  HSVCT currentColor = rgbled.getCurrentColor();
  int oldhue = currentColor.hue;
  currentColor.hue += value;
  if (currentColor.hue > RGBWW_CALC_HUEWHEELMAX)
  {
    currentColor.hue = 0;
  }else if (currentColor.hue < 0){
    currentColor.hue = RGBWW_CALC_HUEWHEELMAX;
  }
  //currentColor.hue = constrain(currentColor.hue, 0, RGBWW_CALC_HUEWHEELMAX);
  //if (oldhue != currentColor.hue)
  setColor(currentColor, 0);
}

bool lightOnHandler(String value) {
//  Serial.print("value: ");
//  Serial.println(value);
  HSVCT color;
  if (value == "true") {
    setColor(colorWhite);
  } else if (value == "false") {
    setColor(colorBlack);
  }
  return true;
}

void setColor(HSVCT color)
{
  setColor(color, TRANSITION_DEFAULT);
}
void setColor(HSVCT color, int fade)
{
  HSVCT currentColor = rgbled.getCurrentColor();
  if (!isSameColor(color, currentColor))
    rgbled.fadeHSV(color, fade);
}

bool isSameColor(const HSVCT& color1, const HSVCT& color2)
{
  if (color1.h != color2.h || color1.s != color2.s || color1.v != color2.v)
    return false;
  else
    return true;
}
#ifdef ALLOW_RGB_MQTT
// {"r": 255, "g": 255, "b":255}
// {"r": 65535, "g": 65535, "b":65535}

bool rgbSetHandler(String value)
{
  int rgbw[5] = {0, 0, 0,0,0};
  StaticJsonBuffer<200> jsonBuffer;
  //delay(1); //short break for esp to catch up
  JsonObject& root = jsonBuffer.parseObject(value);
  //delay(1); //short break for esp to catch up
  if (!root.success()){
    Serial.println("Bad JSON");
    return false;
  }
  rgbw[0] = root["r"];
  rgbw[1] = root["g"];
  rgbw[2] = root["b"];
  rgbw[3] = root["ww"];
  rgbw[4] = root["cw"];
  Serial.println("Setting Values from JSON.");
  rgbled.setOutputRaw( rgbw[0], rgbw[1], rgbw[2], rgbw[3], rgbw[4]);
  return true;
}
#endif
bool isLightOn()
{
  HSVCT c = rgbled.getCurrentColor();
  return ( c.v != 0);
}

void SetAsRadian(HSVCT& hsvct) {
    hsvct.h = (float(hsvct.h) / float(RGBWW_CALC_HUEWHEELMAX)) * 360.0;
    //delay(1); //short break for esp to catch up
    hsvct.s = (float(hsvct.s) / float(RGBWW_CALC_MAXVAL)) * 100.0;
    //delay(1); //short break for esp to catch up
    hsvct.v = (float(hsvct.v) / float(RGBWW_CALC_MAXVAL)) * 100.0;
    //delay(1); //short break for esp to catch up
}

#ifdef ALLOW_HSV_MQTT
bool hsvSetHandler(String value){
  float hue, sat, val = 0;
  int fade = 0;
  HSVCT color;
  StaticJsonBuffer<200> jsonBuffer;
  //delay(1); //short break for esp to catch up
  JsonObject& root = jsonBuffer.parseObject(value);
  //delay(1); //short break for esp to catch up
  if (!root.success()){
    Serial.println("Bad JSON");
    return false;
  }
  hue = root["h"];
  sat = root["s"];
  val = root["v"];
  fade = root["fade"];
  //delay(1); //short break for esp to catch up
  color = HSVCT(hue, sat, val);
  //delay(1); //short break for esp to catch up
  setColor(color, fade);
}
#endif

void addToFadeQueue(HSVCT& color, int time)
{
  rgbled.fadeHSV(color, time, 0, true);
}

void setup() {

  Serial.begin(115200);
  rgbled.init(REDPIN, GREENPIN, BLUEPIN, WWPIN, CWPIN, 300);
  rgbled.colorutils.setColorMode(RGBWW_COLORMODE::RGBWW);
  rgbled.colorutils.setHSVmodel(RGBWW_HSVMODEL::RAINBOW);
  
  Homie.setFirmware(FW_NAME, FW_VERSION);
  lightNode.subscribe("on", lightOnHandler);
#ifdef ALLOW_RGB_MQTT
  lightNode.subscribe("rgb", rgbSetHandler);
#endif  
#ifdef ALLOW_HSV_MQTT
  lightNode.subscribe("hsv", hsvSetHandler);
#endif
  lightNode.subscribe("hass", hassSetHandler);
  
  Homie.registerNode(lightNode);
  Homie.enableLogging(false);
  Homie.setup();
  doStartUpColors();
  rgbled.setAnimationCallback(onColorChanged);
  attachInterrupt(REMOTE_RF_PIN, handleRfInterrupt, CHANGE);
}

void loop() {
  Homie.loop();
  delay(5);
  rgbled.show();
  delay(5);
  if (receivedCommand.isReady == 1)
  {
    if (REMOTE_ID == receivedCommand.packet.id.remote)
    {
      if ((receivedCommand.count % 5) == 0){
        //Serial.print("RECEIVED COMMAND: 0x"); Serial.println(receivedCommand.packet.id.command, HEX);
        //Serial.print(" Count: "); Serial.println(receivedCommand.count);
        switch(receivedCommand.packet.id.command)
        {
          case BUTTON_RED:
            setColor(colorRed);
            break;
          case BUTTON_GREEN:
            setColor(colorGreen);
            break;
          case BUTTON_POWER_ON:
          case BUTTON_WHITE:
            setColor(colorWhite);
            break;
          case BUTTON_BLUE:
            setColor(colorBlue);
            break;
          case BUTTON_POWER_OFF:
            setColor(colorBlack);
            break;
          case BUTTON_UP:
            adjustBrightness(+10);
            break;
          case BUTTON_DOWN:
            adjustBrightness(-10);
            break;
          case BUTTON_MINUS:
            adjustHue(-10);
            break;
          case BUTTON_PLUS:
            adjustHue(+10);
            break;
        }
      } 
    }
    receivedCommand.isReady = 0;
  }
  delay(5);//short break for esp to catch up
}
