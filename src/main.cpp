#pragma GCC optimize ("Ofast")
#include <M5Unified.h>
#include <ESP32Encoder.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PatternMath.h>
#include "OneButton.h"          //For Button Debounce and Longpress
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <SPI.h>
#include "ui/ui.h"
#include <EEPROM.h>
#include "main.h"
#include <esp_wifi.h>
#include "Preferences.h"      //EEPROM replacement function
#include "language.h"

#ifndef LV_CONF_INCLUDE_SIMPLE
#define LV_CONF_INCLUDE_SIMPLE
#endif

#include <lvgl.h>
#include <esp_timer.h>

constexpr int32_t HOR_RES=320;
constexpr int32_t VER_RES=240;



///////////////////////////////////////////
////
////  To Debug or not to Debug
////
///////////////////////////////////////////

// Uncomment the following line if you wish to print DEBUG info
//#define DEBUG 

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif

// Uncomment the following line if you wish to print DEBUG info
#define DEBUGPRIO 

#ifdef DEBUGPRIO
#define LogDebugPRIO(...) Serial.println(__VA_ARGS__)
#define LogDebugFormattedPRIO(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebugPRIO(...) ((void)0)
#define LogDebugFormattedPRIO(...) ((void)0)
#endif

//#define OFF 0.0
//#define ON 1.0

// Screens 

#define ST_UI_START 0
#define ST_UI_HOME 1

#define ST_UI_MENUE 10
#define ST_UI_PATTERN 11
#define ST_UI_Torqe 12
#define ST_UI_EJECTSETTINGS 13
#define ST_UI_Fist_IT_Settings 14

#define ST_UI_SETTINGS 20

int st_screens = ST_UI_START;



// Menü States

#define M_CONNECT 0
#define HOME 1
#define MENUE 2
#define MENUE2 3
#define TORQE 4
#define PATTERN_MENUE 5
#define PATTERN_MENUE2 6
#define PATTERN_MENUE3 7
#define EJECT_MENUE 20
#define Fist_IT_MENUE 20

int menuestatus = M_CONNECT;


// EEPROM replacement function using Non-volatie memory (NVS)
Preferences m5prf; //initiate an instance of the Preferences library

bool eject_status = false;
bool Fist_IT_status = false;
bool dark_mode = false;
bool vibrate_mode = true;
bool touch_home = false;
bool touch_disabled = false;

// Command States
#define CONN 0
#define SPEED 1
#define DEPTH 2
#define STROKE 3
#define SENSATION 4
#define PATTERN 5
#define TORQE_F 6
#define TORQE_R 7
#define OFF 10
#define ON  11
#define SETUP_D_I 12
#define SETUP_D_I_F 13
#define REBOOT 14

#define CUMSPEED 20
#define CUMTIME 21
#define CUMSIZE   22
#define CUMACCEL  23
#define CUMOFF 10
#define CUMON  11

#define Fist_IT_SPEED 30
#define Fist_IT_ROTATION 31
#define Fist_IT_PAUSE   32
#define Fist_IT_ACCEL  33
#define FISTOFF 10
#define FISTON  11


#define CONNECT 88
#define HEARTBEAT 99

int displaywidth;
int displayheight;
int progheight = 30;
int distheight = 10;
int S1Pos;
int S2Pos;
int S3Pos;
int S4Pos;
bool rstate = false;
int pattern = 2;
char patternstr[20];
bool onoff = false;
int HB = 0;


long speedenc = 0;
long depthenc = 0;
long strokeenc = 0;
long sensationenc = 0;
long torqe_f_enc = 0;
long torqe_r_enc = 0;
long EJECT_t_enc = 0;
long EJECT_si_enc =0;
long EJECT_s_enc = 0;
long EJECT_a_enc = 0;
long encoder4_enc = 0;
long Fist_IT_s_enc = 0;
long Fist_IT_r_enc =0;
long Fist_IT_p_enc = 0;
long Fist_IT_a_enc = 0;

float maxdepthinmm = 400.0;
float speedlimit = 600;
int speedscale = -5;

float speed = 0.0;
float depth = 0.0;
float stroke = 0.0;
float sensation = 0.0;
float torqe_f = 100.0;
float torqe_r = -180.0;
float Eject_time = 0.0;
float Fist_IT_speed = 0.0;
float Fist_IT_rotation = 0.0;
float Fist_IT_pause = 0.0;
float Fist_IT_accel = 0.0;

float EJECT_speed = 0.0;
float EJECT_time = 0.0;
float EJECT_size = 0.0;
float EJECT_accel = 0.0;


unsigned long nowMs;
int  rampMs;
bool rampEnabled = true;
int rampValue;
int rampTime = 75;
int maxRamp = 8;
int encId;
int activeEncId;

bool dynamicStroke = false;

String status_message_text;

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

// Variable to store if sending data was successful
String success;

float out_esp_speed;
float out_esp_depth;
float out_esp_stroke;
float out_esp_sensation;
float out_esp_pattern;
bool out_esp_rstate;
bool out_esp_connected;
int out_esp_command;
float out_esp_value;
int out_esp_target;
int out_esp_sender;

float incoming_esp_speed;
float incoming_esp_depth;
float incoming_esp_stroke;
float incoming_esp_sensation;
float incoming_esp_pattern;
bool incoming_esp_rstate;
bool incoming_esp_connected;
bool incoming_esp_heartbeat;
int incoming_esp_target;
int incoming_esp_sender;

typedef struct struct_message {
  float esp_speed;
  float esp_depth;
  float esp_stroke;
  float esp_sensation;
  float esp_pattern;
  bool esp_rstate;
  bool esp_connected;
  bool esp_heartbeat;
  int esp_command;
  float esp_value;
  int esp_target;
  int esp_sender;
} struct_message;

bool Ossm_paired = false;
bool Eject_paired = false;
bool Fist_IT_paired = false;

struct_message outgoingcontrol;
struct_message incomingcontrol;

esp_now_peer_info_t peerInfo;
uint8_t DEFAULT_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s, upon connection gets updated to the actual address
//uint8_t OSSM_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s, upon connection gets updated to the actual address
uint8_t EJECT_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s, upon connection gets updated to the actual address
uint8_t FIST_IT_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast to all ESP32s, upon connection gets updated to the actual address
uint8_t OSSM_Address[] = {0xC0, 0x5D, 0x89, 0xB3, 0xDA, 0xFC}; // Broadcast to all ESP32s, upon connection gets updated to the actual address

#define HEARTBEAT_INTERVAL 5000/portTICK_PERIOD_MS	// 5 seconds

// Bool

bool EJECT_On = false;
bool OSSM_On = false;
bool FIST_On = false;

#define EEPROM_SIZE 200

// Tasks:

TaskHandle_t eRemote_t  = nullptr;  // Esp Now Remote Task
void espNowRemoteTask(void *pvParameters); // Handels the EspNow Remote

bool connectbtn(); //Handels Connectbtn

// Makes vibration motor go Brrrrr
void vibrate(int vbr_Intensity = 200, int vbr_Length = 100){
    if(lv_obj_has_state(ui_vibrate, LV_STATE_CHECKED) == 1){
      M5.Power.setVibration(vbr_Intensity);
      vTaskDelay(vbr_Length);
      M5.Power.setVibration(0);
    }
}

void mxclick();
bool mxclick_short_waspressed = false;
void mxlong();
bool mxclick_long_waspressed = false;
void click2();
bool click2_short_waspressed = false;
void c2long();
bool click2_long_waspressed = false;
void c2double();
bool click2_double_waspressed = false;
void click3();
bool click3_short_waspressed = false;
void c3long();
bool click3_long_waspressed = false;
void c3double();
bool click3_double_waspressed = false;



void StatusMessage(const uint8_t * mac); // Shows status message in debug
void StatusMessageOut(); // Shows status message in debug
void CheckAllPeers();
void ConnectToOSSM(const uint8_t * mac);
void ConnectToEject(const uint8_t * mac);
void ConnectToFist_IT(const uint8_t * mac);
void HandleReceivedOSSM(const uint8_t * mac, const uint8_t *incomingData, int len);
void HandleReceivedEJECT(const uint8_t * mac, const uint8_t *incomingData, int len);
void HandleReceivedFist_IT(const uint8_t * mac, const uint8_t *incomingData, int len);

lv_display_t *display;
lv_indev_t *indev;

static lv_draw_buf_t *draw_buf1;
static lv_draw_buf_t *draw_buf2;

// Display flushing
void my_display_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lv_draw_sw_rgb565_swap(px_map, w*h);
  M5.Display.pushImageDMA<uint16_t>(area->x1, area->y1, w, h, (uint16_t *)px_map);
  lv_disp_flush_ready(disp);
}

uint32_t my_tick_function() {
  return (esp_timer_get_time() / 1000LL);
}

void my_touchpad_read(lv_indev_t * drv, lv_indev_data_t * data) {
  M5.update();
  auto count = M5.Touch.getCount();

  if(touch_disabled != true){
    if ( count == 0 ) {
      data->state = LV_INDEV_STATE_RELEASED;
    } else {
      auto touch = M5.Touch.getDetail(0);
      data->state = LV_INDEV_STATE_PRESSED; 
      data->point.x = touch.x;
      data->point.y = touch.y;
    }
}
}

static void event_cb(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *label = reinterpret_cast<lv_obj_t *>(lv_event_get_user_data(e));

  switch (code)
  {
  case LV_EVENT_ROTARY:
    LogDebugPRIO("Rotary event detected");
    lv_label_set_text(label, "Rotation:\nLV_EVENT_ROTARY");
    break;
  case LV_EVENT_PRESSED:
    lv_label_set_text(label, "The last button event:\nLV_EVENT_PRESSED");
    break;
  case LV_EVENT_CLICKED:
    lv_label_set_text(label, "The last button event:\nLV_EVENT_CLICKED");
    break;
  case LV_EVENT_LONG_PRESSED:
    lv_label_set_text(label, "The last button event:\nLV_EVENT_LONG_PRESSED");
    break;
  case LV_EVENT_LONG_PRESSED_REPEAT:
    lv_label_set_text(label, "The last button event:\nLV_EVENT_LONG_PRESSED_REPEAT");
    break;
  default:
    break;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  LogDebug("OnDataSENT Packet to: ");
  LogDebug(macStr);
  LogDebug(" send status:\t");
  LogDebug("  Target: ");
  LogDebug(outgoingcontrol.esp_target);
  LogDebug("  Command: ");
  LogDebug(outgoingcontrol.esp_command);
  LogDebug("  Value: ");
  LogDebug(outgoingcontrol.esp_value);
  LogDebug(status == ESP_NOW_SEND_SUCCESS ? "  Delivery Success" : "Delivery is Failed");


if (!outgoingcontrol.esp_command==99){
//StatusMessageOut();
}
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
 //CheckAllPeers();
  memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));
  LogDebugFormatted("Incoming message from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);


  if(esp_now_is_peer_exist(mac) and mac != DEFAULT_Address){
    LogDebug("This MAC is allready added in the peers list");
  }
  if(!esp_now_is_peer_exist(mac)){
    LogDebug("Sender is not yet in the peers list");
      // Add the new peer
      memcpy(peerInfo.peer_addr, mac, 6);

     esp_err_t result = esp_now_add_peer(&peerInfo);
  }
  
/*LogDebug("");
LogDebug("*********************");
LogDebug("On Data Received");
LogDebug("OSSM Paired: ");
LogDebug(Ossm_paired);
LogDebug("EJECT Paired: ");
LogDebug(Eject_paired);
LogDebug("FIST-IT Paired: ");
LogDebug(Fist_IT_paired);
LogDebug("Sender: ");
LogDebug(incomingcontrol.esp_sender);
LogDebug("*********************");
LogDebug("");
*/

LogDebug(incomingcontrol.esp_sender);
  memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));
  int ToDo=incomingcontrol.esp_sender;
  if(ToDo==OSSM_ID){
    LogDebug("To OSSM handler");
    HandleReceivedOSSM(mac, incomingData, len);} 
  else if(ToDo==EJECT_ID){
    LogDebug("To EJECT handler");
    HandleReceivedEJECT(mac, incomingData, len);}
  else if(ToDo==FIST_ID){
    LogDebug("To FIST-IT handler");
    HandleReceivedFist_IT(mac, incomingData, len);}
  else{ 
    LogDebug("Unknown sender ID");  
  }
}

void HandleReceivedOSSM(const uint8_t * mac, const uint8_t *incomingData, int len){

    memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));
    LogDebugPRIO("HANDLE RECEIVED OSSM to target ID: ");
    LogDebugPRIO(incomingcontrol.esp_target);
    LogDebugPRIO(" from sender ");
  LogDebugPRIO(incomingcontrol.esp_sender);
if(incomingcontrol.esp_sender==OSSM_ID)  {
  LogDebugPRIO("Received from OSSM");
  LogDebugPRIO("Received Command: ");
    LogDebugPRIO(incomingcontrol.esp_command);
    LogDebugPRIO("Received value: ");
    LogDebugPRIO(incomingcontrol.esp_value);
    LogDebugPRIO("Received to target ID: ");
    LogDebugPRIO(incomingcontrol.esp_target);
    LogDebugPRIO("Received from sender: ");
    LogDebugPRIO(incomingcontrol.esp_sender);
    LogDebugFormattedPRIO("from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if(!Ossm_paired){
      LogDebugPRIO("OSSM is not paired");}
    else{
      LogDebugFormattedPRIO("OSSM paired and ready. OSSM Remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
    }
    
    if(incomingcontrol.esp_target == M5_ID  && 
       Ossm_paired == false){
  
      // Remove the existing peer (0xFF:0xFF:0xFF:0xFF:0xFF:0xFF)
      //esp_err_t result = esp_now_del_peer(peerInfo.peer_addr);
      esp_err_t result = esp_now_del_peer(OSSM_Address);
  
      if (result == ESP_OK) {
  
        memcpy(OSSM_Address, mac, 6); //get the mac address of the sender
        
        // Add the new peer
        memcpy(peerInfo.peer_addr, OSSM_Address, 6);
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
          LogDebugFormattedPRIO("New peer added successfully, OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
          Ossm_paired = true;
        }
        else {
          LogDebugPRIO("Failed to add OSSM as new peer");
        }
      }
      else {
        LogDebugPRIO("Failed to remove peer");
      }
  
      if(incomingcontrol.esp_speed > speedlimit){
        speedlimit = 600;
      } else (
      speedlimit = incomingcontrol.esp_speed);
      LogDebug(speedlimit);
      maxdepthinmm = incomingcontrol.esp_depth;
      LogDebug(maxdepthinmm);
      pattern = incomingcontrol.esp_pattern;
      LogDebug(pattern);
      outgoingcontrol.esp_target = OSSM_ID;
      outgoingcontrol.esp_sender = M5_ID;
      //outgoingcontrol.esp_command = HEARTBEAT;
  
  
      result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      LogDebug(result);
      
      if (result == ESP_OK) {
        Ossm_paired = true;
        LogDebug("OSSM Connected");
        lv_label_set_text(ui_connect, "Connected");
        lv_scr_load_anim(ui_Home, LV_SCR_LOAD_ANIM_FADE_ON,20,0,false);
        }
    }

    if(incomingcontrol.esp_sender==OSSM_ID){
      LogDebug("OSSM handle command");
      switch(incomingcontrol.esp_command)
      {
      case OFF: 
      {
      OSSM_On = false;
      }
      break;
      case ON:
      {
      OSSM_On = true;
      }
      }
    }
  return;
}
}

void HandleReceivedEJECT(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));
/*  LogDebug("HANDLE RECEIVED EJECT to target ID: ");
  LogDebug(incomingcontrol.esp_target);
  LogDebug(" from sender ");
  LogDebug(incomingcontrol.esp_sender);*/
if(incomingcontrol.esp_sender==EJECT_ID){
  /*LogDebug("Received eject");
  LogDebug("Received Command: ");
  LogDebug(incomingcontrol.esp_command);
  LogDebug("Received value: ");
  LogDebug(incomingcontrol.esp_value);
  LogDebug("Received to target ID: ");
  LogDebug(incomingcontrol.esp_target);
  LogDebug("Received from sender: ");
  LogDebug(incomingcontrol.esp_sender);
  LogDebugFormatted("from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);*/
  if(!Eject_paired){
    LogDebug("EJECT is not paired");}
  else{
    LogDebugFormatted("EJECT paired and ready. EJECT Remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
  }
  
  if(incomingcontrol.esp_target == M5_ID  && incomingcontrol.esp_sender == EJECT_ID &&
     Eject_paired == false){
    // Remove the existing peer (0xFF:0xFF:0xFF:0xFF:0xFF:0xFF)
    esp_err_t result = esp_now_del_peer(mac);

    if (result == ESP_OK) {

      memcpy(EJECT_Address, mac, 6); //get the mac address of the sender
      
      // Add the new peer
      memcpy(peerInfo.peer_addr, EJECT_Address, 6);
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        LogDebugFormatted("New peer added successfully, EJECT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
        Eject_paired = true;
      }
      else {
        LogDebug("Failed to add EJECT as new peer");
      }
    }
    else {
      LogDebug("Failed to REMOVE EJECT as peer");
    }

    outgoingcontrol.esp_target = EJECT_ID;
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_command = HEARTBEAT;


    result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    LogDebug(result);
    
    if (result == ESP_OK) {
      Eject_paired = true;
      LogDebug("EJECT Connected");
    }
  }

if(incomingcontrol.esp_sender==EJECT_ID){
  LogDebug("Eject handle command");
  switch(incomingcontrol.esp_command)
  {
  case OFF: 
  {
    EJECT_On = false;
    lv_label_set_text(ui_EJECTButtonLText, "Start");
    lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);  
  }
  break;
  case ON:
  {
  //continue ejectcreampie;
  }
  }
}
}
}

void HandleReceivedFist_IT(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));
/*  LogDebug("HANDLE RECEIVED FIST-IT to target ID: ");
  LogDebug(incomingcontrol.esp_target);
  LogDebug(" from sender ");
  LogDebug(incomingcontrol.esp_sender);
*/
  if(incomingcontrol.esp_sender=FIST_ID){
/*  LogDebug("Received FIST-IT");
  LogDebug("Received Command: ");
  LogDebug(incomingcontrol.esp_command);
  LogDebug("Received value: ");
  LogDebug(incomingcontrol.esp_value);
  LogDebug("Received to target ID: ");
  LogDebug(incomingcontrol.esp_target);
  LogDebug("Received from sender: ");
  LogDebug(incomingcontrol.esp_sender);
  LogDebugFormatted("from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  */



  if(!Fist_IT_paired){
    LogDebug("Fist_IT is not paired");}
  else{
    LogDebugFormatted("Fist_IT paired and ready. Fist_IT Remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
  }
  
  if(incomingcontrol.esp_sender == FIST_ID  && incomingcontrol.esp_target == M5_ID && 
     !Fist_IT_paired){

    // Remove the existing peer (0xFF:0xFF:0xFF:0xFF:0xFF:0xFF)
    esp_err_t result = esp_now_del_peer(mac);

    if (result == ESP_OK) {

      memcpy(FIST_IT_Address, mac, 6); //get the mac address of the sender
      
      // Add the new peer
      memcpy(peerInfo.peer_addr, FIST_IT_Address, 6);
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        LogDebugFormatted("New peer added successfully, FIST-IT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
        Fist_IT_paired = true;
      }
      else {
        LogDebug("Failed to add new peer");
      }
    }
    else {
      LogDebug("Failed to remove peer");
    }

    outgoingcontrol.esp_target = FIST_ID;
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_command = HEARTBEAT;


    result = esp_now_send(FIST_IT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    LogDebug(result);
    
    if (result == ESP_OK) {
      Fist_IT_paired = true;
      LogDebug("Fist_IT Connected");
    }
  }

//end connection tries

  if(incomingcontrol.esp_sender==FIST_ID){
    LogDebug("Fist-IT handle command");
    switch(incomingcontrol.esp_command)
    {
    case OFF: 
    {
      FIST_On = false;
      //lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);  
    }
    break;
    case ON:
    {
    //continue Fist-IT;
    }
    }
  }
  
  return;
} 
}

void setup(){
  /*auto cfg = M5.config();
  M5.begin(cfg);
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(115200);
  M5.Power.setChargeCurrent(BATTERY_CHARGE_CURRENT);
  LogDebug("\n Starting");      // Start LogDebug 

  //****Load EEPROOM:
  eject_status = EEPROM.readBool(EJECT);
  Fist_IT_status = EEPROM.readBool(FIST);
  dark_mode = EEPROM.readBool(DARKMODE);
  vibrate_mode = EEPROM.readBool(VIBRATE);
  touch_home = EEPROM.readBool(LEFTY);
  */
Serial.begin(115200);
    auto cfg = M5.config();

    M5.begin(cfg);

  m5prf.begin("m5-ctnr", false); 
    // Loads these settings at boot
    eject_status = m5prf.getBool("ejectAddon", false); //boolean here is used if key does not exist yet
    dark_mode = m5prf.getBool("Darkmode", true);       // ^ (basically first boot defaults, saving settings surives a re-flash!)
    Fist_IT_status = m5prf.getBool("Fist-ITAddon", false);
    vibrate_mode = m5prf.getBool("Vibrate", true);
    touch_home= m5prf.getBool("Lefty", false);       // = touchcreen. There apears to be no actual lefthanded mode anywhere
  m5prf.end();

//SET BUTTONS
//    if (!eject_status){
//      lv_obj_add_state(ui_HomeButtonL, LV_STATE_DISABLED);}
//    else {
//      lv_obj_add_state(ui_HomeButtonL, LV_STATE_DEFAULT);}

//END BUTTONS

  M5.Power.setChargeCurrent(BATTERY_CHARGE_CURRENT);
  LogDebug("\n Starting");      // Start LogDebug

  WiFi.disconnect(true); // Ensure not connected to any AP
  WiFi.mode(WIFI_STA);

  int32_t channel = 1; // getWiFiChannel(WIFI_SSID);

  Serial.print("WiFi Channel before: " + WiFi.channel());
  //WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
//  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("WiFi Channel after: " + WiFi.channel());
  //WiFi.printDiag(Serial); // Uncomment to verify channel change after
  
  LogDebug(WiFi.macAddress());
  LogDebug("");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    LogDebug("Error initializing ESP-NOW");
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, OSSM_Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    LogDebug("Failed to add OSSM as peer");
  }
  else{LogDebug("OSSM added as peer succesfully");
      LogDebugFormatted("MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
  }
  delay(200);
  if(eject_status==true){  //was true, not 1234
  // register second peer  
  memcpy(peerInfo.peer_addr, EJECT_Address, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    LogDebug("Failed to add EJECT as peer");
  }
  else{LogDebug("EJECT added as peer succesfully");
    LogDebugFormatted("MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
  }
  }
  else{
    LogDebug("Eject not initially added");
  }
  delay(200);
  Fist_IT_status=true;
  if(Fist_IT_status==true){  //was ==true, not 1234
    // register second peer  
    memcpy(peerInfo.peer_addr, FIST_IT_Address, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      LogDebug("Failed to add Fist-IT as peer");
    }
    else{LogDebug("FIST-IT added as peer succesfully");
      LogDebugFormatted("MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
    }
    }
    else{
      LogDebug("FIST-IT not initially added");
    }
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  xTaskCreatePinnedToCore(espNowRemoteTask,      /* Task function. */
                            "espNowRemoteTask",  /* name of task. */
                            3096,               /* Stack size of task */
                            NULL,               /* parameter of the task */
                            5,                  /* priority of the task */
                            &eRemote_t,         /* Task handle to keep track of created task */
                            0);                 /* pin task to core 0 */
  delay(200);

//  xTaskCreatePinnedToCore(espNowRemoteTaskEject,      /* Task function. */
//    "espNowRemoteTaskEject",  /* name of task. */
//    3096,               /* Stack size of task */
//    NULL,               /* parameter of the task */
//    5,                  /* priority of the task */
//    &eRemote_t_EJECT,         /* Task handle to keep track of created task */
//    0);                 /* pin task to core 0 */
//delay(200);
 
  encoder1.attachHalfQuad(ENC_1_CLK, ENC_1_DT);
  encoder2.attachHalfQuad(ENC_2_CLK, ENC_2_DT);
  encoder3.attachHalfQuad(ENC_3_CLK, ENC_3_DT);
  encoder4.attachHalfQuad(ENC_4_CLK, ENC_4_DT);
  Button1.attachClick(mxclick);
  Button1.attachLongPressStop(mxlong);
  Button2.attachClick(click2);
  Button2.attachDoubleClick(c2double);
  Button2.attachLongPressStop(c2long);  
  Button3.attachClick(click3);
  Button3.attachMultiClick(c3double);
  Button3.attachLongPressStop(c3long);  


  // Initialize `disp_buf` display buffer with the buffer(s).
  // lv_draw_buf_init(&draw_buf, LV_HOR_RES_MAX, LV_VER_RES_MAX);
  M5.Display.setEpdMode(epd_mode_t::epd_fastest); // fastest but very-low quality.
  if (M5.Display.width() < M5.Display.height())
  { /// Landscape mode.
  M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  }
  
  lv_init();
  lv_tick_set_cb(my_tick_function);

  display = lv_display_create(HOR_RES, VER_RES);
  lv_display_set_flush_cb(display, my_display_flush);

  static lv_color_t buf1[HOR_RES * 15]; 
  lv_display_set_buffers(display, buf1, nullptr, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

  indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);
  ui_init();  

  if(eject_status == true){
  lv_obj_add_state(ui_ejectaddon, LV_STATE_CHECKED);
  lv_obj_clear_state(ui_EJECTSettingButton, LV_STATE_DISABLED);
  lv_obj_clear_state(ui_HomeButtonL, LV_STATE_DISABLED);
  }
  if(Fist_IT_status== true){
    lv_obj_add_state(ui_Fist_IT_addon, LV_STATE_CHECKED);
  }
    if(dark_mode == true){
  lv_obj_add_state(ui_darkmode, LV_STATE_CHECKED);
  }
  if(vibrate_mode == true){
  lv_obj_add_state(ui_vibrate, LV_STATE_CHECKED);
  }
  if(touch_home == true){
  lv_obj_add_state(ui_lefty, LV_STATE_CHECKED);
  }
  lv_roller_set_selected(ui_PatternS,2,LV_ANIM_OFF);
  lv_roller_get_selected_str(ui_PatternS,patternstr,0);
  lv_label_set_text(ui_HomePatternLabel,patternstr);


}

//Sends Commands and Value to Remote device returns ture or false if sended
bool SendCommand(int Command, float Value, int Target){
    //  StatusMessageOut();
//CheckAllPeers();
LogDebug("SENDCOMMAND");
LogDebug(Target);
LogDebug("OSSM Paired: ");
LogDebug(Ossm_paired);

outgoingcontrol.esp_sender = M5_ID;
//if(outgoingcontrol.esp_target == OSSM_ID){
if(Target == OSSM_ID){
  LogDebug("Send to OSSM - step 1");

  if(Ossm_paired == true){
    LogDebug("OSSM PAIRED WHEN SENDING");
    outgoingcontrol.esp_connected = true;
    outgoingcontrol.esp_command = Command;
    outgoingcontrol.esp_value = Value;
    outgoingcontrol.esp_target = OSSM_ID;
    esp_err_t result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    LogDebug("Send to OSSM - step 2");
  
    if (result == ESP_OK) {
      return true;
      LogDebug("Sending to ossm was successfull");
    } 
    else {
      // Fail fast without retry to avoid blocking UI thread
      LogDebug("Sending to OSSM failed");
      // Retry commented out to prevent watchdog timeout - uncomment if needed
      //outgoingcontrol.esp_target = OSSM_ID;
      //esp_err_t result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      return false;
    }
  }
//  LogDebug("Send to OSSM - step 5");
    if(!Ossm_paired){
      LogDebug("Trying to Pair OSSM again");
//      LogDebug("OSSM not paired, sending Heartbeat for OSSM");
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = OSSM_ID;
      outgoingcontrol.esp_sender = M5_ID;
      esp_err_t result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
//      LogDebug("Send to OSSM - step 7");
      return false;
    }
  }

return true;
}

//Sends Commands and Value to Remote device returns ture or false if sended
bool SendCumCommand(int Command, float Value, int Target){
/*
  if(!Eject_paired){
    //  StatusMessageOut();
    memcpy(peerInfo.peer_addr, EJECT_Address, 6);
    esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    LogDebugFormatted("New peer added successfully, EJECT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
    Eject_paired = true;
  }
  else {
    LogDebug("Failed to add new peer");
  }
  }
*/

  LogDebug("SEND CUM COMMAND");
  //CheckAllPeers();
  LogDebug("Target: ");
  LogDebug(Target);
  LogDebug("Command: ");
  LogDebug(Command);
  LogDebug("Value: ");
  LogDebug(Value);
  LogDebug("Eject Paired: ");
  LogDebug(Eject_paired);
  LogDebugFormatted("EJECT MAC : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);

outgoingcontrol.esp_sender = M5_ID;

if(Target == EJECT_ID){
LogDebug("Send to Eject - step 1");
  if(Eject_paired == true){
      LogDebug("EJECT PAIRED WHEN SENDING");
    outgoingcontrol.esp_connected = true;
    outgoingcontrol.esp_command = Command;
    outgoingcontrol.esp_value = Value;
    outgoingcontrol.esp_target = EJECT_ID;
    outgoingcontrol.esp_sender = M5_ID;
    LogDebugFormatted("Sending to EJECT MAC : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);

    esp_err_t result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      LogDebug("Send to Eject - step 2");
  
    if (result == ESP_OK) {
      LogDebug("Command sent to EJECT: ");
      LogDebug(outgoingcontrol.esp_command);
          return true;
    } 
    else {
      // Fail fast without retry to avoid blocking UI thread
      LogDebug("Sending to eject failed");
      // Retry commented out to prevent watchdog timeout - uncomment if needed
      //outgoingcontrol.esp_target = EJECT_ID;
      //esp_err_t result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      return false;
    }

  }
  else{
    // Heartbeat removed from UI loop to prevent watchdog timeout
    // Background task (espNowRemoteTask) handles heartbeats every 5 seconds
    LogDebug("Eject was not paired - heartbeat handled by background task");
    //CheckAllPeers();

    /* HEARTBEAT COMMENTED OUT - causes watchdog timeout when called from UI loop
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_command = HEARTBEAT;
    outgoingcontrol.esp_value = 0;
    outgoingcontrol.esp_target = EJECT_ID;
    esp_err_t result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    if (result == ESP_OK) {
      LogDebug(" Heartbeat to EJECT is sent for connection: ");
      LogDebug(outgoingcontrol.esp_command);
          return true;
    } 
    else {
      LogDebug("Heartbeat to eject failed");
      return false;
    }
    */
    return false;
  }

}
return true;

}

bool SendFistCommand(int Command, float Value, int Target){
/*
  if(!Fist_IT_paired){
    //  StatusMessageOut();
    memcpy(peerInfo.peer_addr, FIST_IT_Address, 6);
    esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result == ESP_OK) {
    LogDebugFormatted("New peer added successfully, OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
  }
  else {
    LogDebug("Failed to add new peer");
  }

        }
*/
  LogDebug("SEND FIST COMMAND");
  //CheckAllPeers();
  LogDebug("Target: ");
  LogDebug(Target);
  LogDebug("Command: ");
  LogDebug(Command);
  LogDebug("Value: ");
  LogDebug(Value);
  LogDebug("Fist_IT Paired: ");
  LogDebug(Fist_IT_paired);
  LogDebugFormatted("Fist_IT MAC : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);

outgoingcontrol.esp_sender = M5_ID;

if(Target == FIST_ID){
LogDebug("Send to Fist_IT - step 1");
  if(Fist_IT_paired == true){
      LogDebug("Fist_IT PAIRED WHEN SENDING");
    outgoingcontrol.esp_connected = true;
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_command = Command;
    outgoingcontrol.esp_value = Value;
    outgoingcontrol.esp_target = FIST_ID;
    LogDebugFormatted("Sending to Fist_IT MAC : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);

    esp_err_t result = esp_now_send(FIST_IT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      LogDebug("Send to Fist_IT - step 2");
  
    if (result == ESP_OK) {
      LogDebug("Command sent to Fist_IT: ");
      LogDebug(outgoingcontrol.esp_command);
          return true;
    } 
    else {
      // Fail fast without retry to avoid blocking UI thread
      LogDebug("Sending to Fist_IT failed");
      // Retry commented out to prevent watchdog timeout - uncomment if needed
      //outgoingcontrol.esp_target = FIST_ID;
      //esp_err_t result = esp_now_send(FIST_IT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      return false;
    }

  }
  else{
    // Heartbeat removed from UI loop to prevent watchdog timeout
    // Background task (espNowRemoteTask) handles heartbeats every 5 seconds
    LogDebug("Fist_IT was not paired - heartbeat handled by background task");
    //CheckAllPeers();

    /* HEARTBEAT COMMENTED OUT - causes watchdog timeout when called from UI loop
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_command = HEARTBEAT;
    outgoingcontrol.esp_value = 0;
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_target = FIST_ID;
    esp_err_t result = esp_now_send(FIST_IT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    if (result == ESP_OK) {
      LogDebug(" Heartbeat to FIST-IT is sent for connection: ");
      LogDebug(outgoingcontrol.esp_command);
          return true;
    } 
    else {
      LogDebug("Heartbeat to Fist_IT failed");
      return false;
    }
    */
    return false;
  }

}
return true;

}

void connectbutton(lv_event_t * e)
{
  LogDebugPRIO("CONNECTBUTTON PRESSED");
    if(!Ossm_paired){
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = M5_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = OSSM_ID;
      esp_err_t result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      if(result == ESP_OK){
        LogDebugFormattedPRIO("Heartbeat to OSSM sent for connection, MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X on channel \n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5],WiFi.channel());
    }
  } else{
    LogDebugPRIO("OSSM already paired");
  }
}

void savesettings(lv_event_t * e)
{

  m5prf.begin("m5-ctnr", false); //open NVS-storage container/session. False means that it's used it in read+write mode. Set true to open or create the namespace in read-only mode.

  if(lv_obj_has_state(ui_vibrate, LV_STATE_CHECKED) == 1){
    m5prf.putBool("Vibrate", true); //NSV-storage write true to key "Vibrate"
    vibrate_mode = true;
	}else if(lv_obj_has_state(ui_vibrate, LV_STATE_CHECKED) == 0){
    vibrate_mode = false;
    m5prf.putBool("Vibrate", false);
	}

  if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
    m5prf.putBool("Lefty", true); // ui_lefty in SL-Studio code is actually Touch-enable toggle
    touch_home = true;
	}else if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 0){
    m5prf.putBool("Lefty", false);
    touch_home = false;
  }

	if(lv_obj_has_state(ui_ejectaddon, LV_STATE_CHECKED) == 1){
    m5prf.putBool("ejectAddon", true);  
    eject_status = true;
	}else if(lv_obj_has_state(ui_ejectaddon, LV_STATE_CHECKED) == 0){
    m5prf.putBool("ejectAddon", false);
    eject_status = false;
  }

	if(lv_obj_has_state(ui_Fist_IT_addon, LV_STATE_CHECKED) == 1){
    m5prf.putBool("Fist-ITAddon", true);  
    Fist_IT_status = true;
  }else if(lv_obj_has_state(ui_Fist_IT_addon, LV_STATE_CHECKED) == 0){
    m5prf.putBool("Fist-ITAddon", false);
    Fist_IT_status = false;
  }

  //read darkmode saved setting to force reboot for theme change
  bool theme_Change_Previous = false;
  bool theme_Change_New = false;
  theme_Change_Previous = m5prf.getBool("Darkmode", true);

  if(lv_obj_has_state(ui_darkmode, LV_STATE_CHECKED) == 1){
    theme_Change_New = true;
    m5prf.putBool("Darkmode", true);
    dark_mode = true;
	}else if(lv_obj_has_state(ui_darkmode, LV_STATE_CHECKED) == 0){
    theme_Change_New = false;
    m5prf.putBool("Darkmode", false);
    dark_mode = false;  
  }

  m5prf.end(); //close storage container/session.
  delay(100);

  if(theme_Change_Previous != theme_Change_New){
    vibrate(225,75);
    ESP.restart(); //reboot is only required to change themes, you don't need to restart for settings to save with NVS.
  }else{
    vibrate(225,75);
  }
}

void screenmachine(lv_event_t * e)
{
  if (lv_scr_act() == ui_Start){
    st_screens = ST_UI_START;
  }
   else if (lv_scr_act() == ui_Home){
    st_screens = ST_UI_HOME;
    speed = lv_slider_get_value(ui_homespeedslider);
    lv_slider_set_range(ui_homedepthslider, 0, maxdepthinmm);
    lv_slider_set_range(ui_homestrokeslider, 0, maxdepthinmm);

  } else if (lv_scr_act() == ui_Menue){
    st_screens = ST_UI_MENUE;

  } else if (lv_scr_act() == ui_Pattern){
    st_screens = ST_UI_PATTERN;

  } else if (lv_scr_act() == ui_Torqe){
    st_screens = ST_UI_Torqe;
    torqe_f = lv_slider_get_value(ui_outtroqeslider);
    torqe_f_enc = fscale(50, 200, 0, Encoder_MAP, torqe_f, 0);
    encoder1.setCount(torqe_f_enc);

    torqe_r = lv_slider_get_value(ui_introqeslider);
    torqe_r_enc = fscale(20, 200, 0, Encoder_MAP, torqe_r, 0);
    encoder4.setCount(torqe_r_enc);

  } else if (lv_scr_act() == ui_EJECTSettings){
    st_screens = ST_UI_EJECTSETTINGS;

  } else if (lv_scr_act() == ui_Fist_IT_Settings){
    st_screens = ST_UI_Fist_IT_Settings;

  } else if (lv_scr_act() == ui_Settings){
    st_screens = ST_UI_SETTINGS;

  }
}

void ejectcreampie(lv_event_t * e){
  LogDebug("EJECTCREAMPIE");
  if(EJECT_On == true){
    SendCumCommand(CUMOFF, 0.0, EJECT_ID);
    lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
    lv_label_set_text(ui_EJECTButtonLText, "Cum");   // update label
    EJECT_On = false;
  } else if(EJECT_On == false){
    lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
    SendCumCommand(CUMON, 0.0, EJECT_ID);
    lv_label_set_text(ui_EJECTButtonLText, "Stop");   // update label
    EJECT_On = true;
//lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
  }
}

void Start_Fist(lv_event_t * e){
  LogDebug("START FIST-IT");
  if(FIST_On == true){
    SendFistCommand(FISTOFF, 0.0, FIST_ID);
    FIST_On = false;
    lv_label_set_text(ui_Fist_IT_ButtonLText, "Start");   // update label
    //    lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
  } else if(FIST_On == false){
//    lv_obj_add_state(ui_HomeButtonL, LV_STATE_CHECKED);
    SendFistCommand(FISTON, 0.0, FIST_ID);
    FIST_On = true;
    lv_label_set_text(ui_Fist_IT_ButtonLText, "Stop");   // update label
//lv_obj_clear_state(ui_HomeButtonL, LV_STATE_CHECKED);
  }
}

void savepattern(lv_event_t * e){
  pattern = lv_roller_get_selected(ui_PatternS);
  lv_roller_get_selected_str(ui_PatternS,patternstr,0);
  lv_label_set_text(ui_HomePatternLabel,patternstr);
  LogDebug(pattern);
  float patterns = pattern;
  SendCommand(PATTERN, patterns, OSSM_ID);
}

void homebuttonmevent(lv_event_t * e){
  LogDebugPRIO("HomeButton");
//OSSM Turn on / off
  if(OSSM_On == false){
    SendCommand(ON, 0, OSSM_ID);
    OSSM_On = true;
  } else if(OSSM_On == true){
    SendCommand(OFF, 0, OSSM_ID);
    OSSM_On = false;
    SendCumCommand(CUMOFF, 0.0, EJECT_ID);
    EJECT_On = false;
    SendFistCommand(FISTOFF, 0.0, FIST_ID);
    FIST_On = false;
  }
}

void setupDepthInter(lv_event_t * e){
    SendCommand(SETUP_D_I, 0.0, OSSM_ID);
}

void setupdepthF(lv_event_t * e){
    SendCommand(SETUP_D_I_F, 0.0, OSSM_ID);
}

void loop()
{
     bool changed=false;
     const int BatteryLevel = M5.Power.getBatteryLevel();
     String BatteryValue = (String(BatteryLevel, DEC) + "%");
     const char *battVal = BatteryValue.c_str();
     lv_bar_set_value(ui_Battery, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue, battVal);
     lv_bar_set_value(ui_Battery1, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue1, battVal);
     lv_bar_set_value(ui_Battery2, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue2, battVal);
     lv_bar_set_value(ui_Battery3, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue3, battVal);
     lv_bar_set_value(ui_Battery4, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue4, battVal);
     lv_bar_set_value(ui_Battery5, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue5, battVal);
     lv_bar_set_value(ui_Battery6, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue6, battVal);
     lv_bar_set_value(ui_Battery7, BatteryLevel, LV_ANIM_OFF);
     lv_label_set_text(ui_BattValue7, battVal);

     M5.update();
     Button1.tick();
     Button2.tick();
     Button3.tick();
     lv_task_handler();


     switch(st_screens){
      
     case ST_UI_START: //Menu With logo after boot
      {

       if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }
        //LogDebugPRIO("UiStart");
        if(click2_short_waspressed == true){
         lv_obj_send_event(ui_StartButtonL, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_obj_send_event(ui_StartButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_obj_send_event(ui_StartButtonR, LV_EVENT_SHORT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_HOME: //Menu with OSSM control sliders
      {
        // TRACK CHANGES: Suppress button press if rotation detected for left/right encoder
//        bool //rotationDetectedLeft = false;  // encoder2
//        bool //rotationDetectedRight = false; // encoder4

        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }

          //RampHelper
          nowMs = millis();
          if (nowMs - rampMs <= rampTime && rampEnabled == true){ 
              if (rampValue <= maxRamp && encId == activeEncId){
            				++rampValue;
              }
          }else{
          rampValue = 1;
          activeEncId = encId;
          }

        //
        // Encoder 1 Speed 
        //
        if(lv_slider_is_dragged(ui_homespeedslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_homespeedslider, speed, LV_ANIM_OFF);

          if (encoder1.getCount() >= 2){      //speed up
            changed = true;
            speed += rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
		      }else if (encoder1.getCount() <= -2){      //speed down
            changed = true;
            speed -=rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
          }

          //speed min-max bounds
          if (speed < 0){
            changed = true;
            speed = 0;
          }
          if (speed > speedlimit){
            changed = true;
            speed = speedlimit;
          }
          
          //send speed
          if (changed) {
            SendCommand(SPEED, speed, OSSM_ID);
          }
        
        
        }else if (lv_slider_get_value(ui_homespeedslider) != speed){ //if slider moved
            speed = lv_slider_get_value(ui_homespeedslider);
            SendCommand(SPEED, speed, OSSM_ID);
        }
        char speed_v[7];
        dtostrf(speed, 6, 0, speed_v);
        lv_label_set_text(ui_homespeedvalue, speed_v);

        //
        // Encoder 2 Depth 
        //
        if(lv_slider_is_dragged(ui_homedepthslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_homedepthslider, depth, LV_ANIM_OFF);

          if (encoder2.getCount() >= 2){      //depth up
            changed = true;
            depth += rampValue;
            if (dynamicStroke == true){
              stroke += rampValue;
            }
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
//            //rotationDetectedLeft = true; // TRACK CHANGES
          }else if (encoder2.getCount() <= -2){      //depth down
            changed = true;
            depth -=rampValue;
            if (dynamicStroke == true){
              stroke -= rampValue;
              if(stroke >= depth){
                stroke = depth;
              }
            }
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
            ////rotationDetectedLeft = true; // TRACK CHANGES
          }

          //depth min-max bounds
          if (depth < 0){
            changed = true;
            depth = 0;
          }
          if (depth > maxdepthinmm){
            changed = true;
            depth = maxdepthinmm;
          }
          
          //send depth
          if (changed) {
            SendCommand(DEPTH, depth, OSSM_ID);
          }
        }else if(lv_slider_get_value(ui_homedepthslider) != depth){
            depth = lv_slider_get_value(ui_homedepthslider);
            SendCommand(DEPTH, depth, OSSM_ID);
        }
        char depth_v[7];
        dtostrf(depth, 6, 0, depth_v);
        lv_label_set_text(ui_homedepthvalue, depth_v);
        
        //
        // Encoder 3 Stroke 
        //
        if(lv_slider_is_dragged(ui_homestrokeslider) == false){ //if knob gets rotated
          changed = false;
          lv_bar_set_start_value(ui_homestrokeslider, depth - stroke, LV_ANIM_OFF);
          lv_slider_set_value(ui_homestrokeslider, depth, LV_ANIM_OFF);


		      if (encoder3.getCount() >= 2){      //Stroke up
            changed = true;
            stroke += rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
          }else if (encoder3.getCount() <= -2){      //Stroke down
            changed = true;
            stroke -= rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
          }

          //Stoke min-max bounds
          if (stroke < 0){
            changed = true;
            stroke = 0;
          }
          if (stroke > maxdepthinmm){
            changed = true;
            stroke = maxdepthinmm;
          }
          
          //send stroke
          if (changed) {
            SendCommand(STROKE, stroke, OSSM_ID);
          }

        } else if(lv_slider_get_left_value(ui_homestrokeslider) != depth - stroke){
            stroke = depth - lv_slider_get_left_value(ui_homestrokeslider);
            SendCommand(STROKE, stroke, OSSM_ID);
        } else if(lv_slider_get_value(ui_homestrokeslider) != depth){
            depth = lv_slider_get_value(ui_homestrokeslider);
            SendCommand(DEPTH, depth, OSSM_ID);
        }

        char stroke_v[7];
        dtostrf(stroke, 6, 0, stroke_v);
        lv_label_set_text(ui_homestrokevalue, stroke_v);  //was lv_label_set_text(ui_homestrokevalue, stroke_v);

        //
        // Encoder4 Sensation (RIGHT ENCODER)
        //
        if(lv_slider_is_dragged(ui_homesensationslider) == false){
          changed = false;
          lv_slider_set_value(ui_homesensationslider, sensation, LV_ANIM_OFF);

          if (encoder4.getCount() >= 2){      //Stroke up
            changed = true;
            sensation += 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
            ////rotationDetectedRight = true; // TRACK CHANGES
          }else if (encoder4.getCount() <= -2){      //Stroke down
            changed = true;
            sensation -= 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
            ////rotationDetectedRight = true; // TRACK CHANGES
          }

          //Stoke min-max bounds
          if (sensation < -100){
            changed = true;
            sensation = -100;
          }
          if (sensation > 100){
            changed = true;
            sensation = 100;
          }

          if (changed) {
            SendCommand(SENSATION, sensation, OSSM_ID);
          }          
        } else if(lv_slider_get_value(ui_homesensationslider) != sensation){
            sensation = lv_slider_get_value(ui_homesensationslider);
            SendCommand(SENSATION, sensation, OSSM_ID);
        }


        // Check each button independently - not else-if chain to avoid blocking
        if(click2_long_waspressed == true){
         lv_obj_send_event(ui_HomeButtonL, LV_EVENT_LONG_PRESSED, NULL);
         LogDebugPRIO("Home Long Left");
        }
        if(click2_short_waspressed == true){
         lv_obj_send_event(ui_HomeButtonL, LV_EVENT_SHORT_CLICKED, NULL);
         LogDebugPRIO("Home Short Left");
        }
        if(click2_double_waspressed == true){
          LogDebugPRIO("Home Double Left");
          // what to do when double pressed left button
        }
        if(mxclick_short_waspressed == true){
          LogDebugPRIO("Home Short Middle");
          lv_obj_send_event(ui_HomeButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        }
        if(mxclick_long_waspressed == true){
          LogDebugPRIO("Home Long Middle");
          lv_obj_send_event(ui_HomeButtonM, LV_EVENT_LONG_PRESSED, NULL);
        }
        if(click3_short_waspressed == true){
          LogDebugPRIO("Home Short Right");
          lv_obj_send_event(ui_HomeButtonR, LV_EVENT_SHORT_CLICKED, NULL);
        }
        if(click3_long_waspressed == true){
          LogDebugPRIO("Home Long Right");
          lv_obj_send_event(ui_HomeButtonR, LV_EVENT_LONG_PRESSED, NULL);
        }
        if(click3_double_waspressed == true){
          if (dynamicStroke == false){
            dynamicStroke = true;;            /// dynamicStroke = !dynamicStroke; crashes M5 for some reason
          }else{
            dynamicStroke = false;;
          }
          if (stroke >= depth){
            stroke = depth;
          }
        }
      }
      break;

      case ST_UI_MENUE:
      {
        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }
        if(encoder4.getCount() > encoder4_enc + 2){
          LogDebug("next");
          lv_group_focus_next(ui_g_menue);
          encoder4_enc = encoder4.getCount();
        } else if(encoder4.getCount() < encoder4_enc -2){
          lv_group_focus_prev(ui_g_menue);
          LogDebug("Preview");
          encoder4_enc = encoder4.getCount();
        }

        if(click2_short_waspressed == true){
         lv_obj_send_event(ui_MenueButtonL, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_obj_send_event(ui_MenueButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_obj_send_event(lv_group_get_focused(ui_g_menue), LV_EVENT_SHORT_CLICKED, NULL);
        } else if(click3_long_waspressed == true){
         SendCommand(REBOOT, 0, OSSM_ID);
        } 
      }
      break;

      case ST_UI_PATTERN:
      {
        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }
        if(encoder4.getCount() > encoder4_enc + 2){
          LogDebug("next");
          uint32_t t = LV_KEY_DOWN;
          lv_obj_send_event(ui_PatternS, LV_EVENT_KEY, &t);
          encoder4_enc = encoder4.getCount();
        } else if(encoder4.getCount() < encoder4_enc -2){
          uint32_t t = LV_KEY_UP;
          lv_obj_send_event(ui_PatternS, LV_EVENT_KEY, &t);
          LogDebug("Preview");
          encoder4_enc = encoder4.getCount();
        }
         if(click2_short_waspressed == true){
         lv_obj_send_event(ui_PatternButtonL, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_obj_send_event(ui_PatternButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_obj_send_event(ui_PatternButtonR, LV_EVENT_SHORT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_Torqe:
      {
        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }
        // Encoder 1 Torqe Out
        if(lv_slider_is_dragged(ui_outtroqeslider) == false){
          if (encoder1.getCount() != torqe_f_enc){
            lv_slider_set_value(ui_outtroqeslider, torqe_f, LV_ANIM_OFF);
            if(encoder1.getCount() <= 0){
              encoder1.setCount(0);
            } else if (encoder1.getCount() >= Encoder_MAP){
              encoder1.setCount(Encoder_MAP);
            } 
            torqe_f_enc = encoder1.getCount();
            torqe_f = fscale(0, Encoder_MAP, 50, 200, torqe_f_enc, 0);
            SendCommand(TORQE_F, torqe_f, OSSM_ID);
          }
        } else if(lv_slider_get_value(ui_outtroqeslider) != torqe_f){
            torqe_f_enc = fscale(50, 200, 0, Encoder_MAP, torqe_f, 0);
            encoder1.setCount(torqe_f_enc);
            torqe_f = lv_slider_get_value(ui_outtroqeslider);
            SendCommand(TORQE_F, torqe_f, OSSM_ID);
        }
        char torqe_f_v[7];
        dtostrf((torqe_f*-1), 6, 0, torqe_f_v);
        lv_label_set_text(ui_outtroqevalue, torqe_f_v);

        // Encoder 4 Torqe IN
        if(lv_slider_is_dragged(ui_introqeslider) == false){
          if (encoder4.getCount() != torqe_r_enc){
            lv_slider_set_value(ui_introqeslider, torqe_r, LV_ANIM_OFF);
            if(encoder4.getCount() <= 0){
              encoder4.setCount(0);
            } else if (encoder4.getCount() >= Encoder_MAP){
              encoder4.setCount(Encoder_MAP);
            } 
            torqe_r_enc = encoder4.getCount();
            torqe_r = fscale(0, Encoder_MAP, 20, 200, torqe_r_enc, 0);
            SendCommand(TORQE_R, torqe_r, OSSM_ID);
          }
        } else if(lv_slider_get_value(ui_introqeslider) != torqe_r){
            torqe_r_enc = fscale(20, 200, 0, Encoder_MAP, torqe_r, 0);
            encoder4.setCount(torqe_r_enc);
            torqe_r = lv_slider_get_value(ui_introqeslider);
            SendCommand(TORQE_R, torqe_r, OSSM_ID);
        }
        char torqe_r_v[7];
        dtostrf(torqe_r, 6, 0, torqe_r_v);
        lv_label_set_text(ui_introqevalue, torqe_r_v);

         if(click2_short_waspressed == true){
         lv_obj_send_event(ui_TorqeButtonL, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(mxclick_short_waspressed == true){
         lv_obj_send_event(ui_TorqeButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        } else if(click3_short_waspressed == true){
         lv_obj_send_event(ui_TorqeButtonR, LV_EVENT_SHORT_CLICKED, NULL);
        }
      }
      break;

      case ST_UI_EJECTSETTINGS: //Menu with EJECT control sliders
      {
        // TRACK CHANGES: Detect left/right encoder rotation once per loop
//        bool rotationDetectedLeft = (encoder2.getCount() >= 2 || encoder2.getCount() <= -2);
//        bool rotationDetectedRight = (encoder4.getCount() >= 2 || encoder4.getCount() <= -2);

        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }

          //RampHelper
          nowMs = millis();
          if (nowMs - rampMs <= rampTime && rampEnabled == true){ 
              if (rampValue <= maxRamp && encId == activeEncId){
            				++rampValue;
              }
          }else{
          rampValue = 1;
          activeEncId = encId;
          }

        //
        // Encoder 1 EJECT_speed 
        //
        if(lv_slider_is_dragged(ui_EJECTSPEEDslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_EJECTSPEEDslider, EJECT_speed, LV_ANIM_OFF);

          if (encoder1.getCount() >= 2){      //EJECT_speed up
            changed = true;
            EJECT_speed += rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
		      }else if (encoder1.getCount() <= -2){      //EJECT_speed down
            changed = true;
            EJECT_speed -=rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
          }

          //EJECT_speed min-max bounds
          if (EJECT_speed < 0){
            changed = true;
            EJECT_speed = 0;
          }
          if (EJECT_speed > 100){
            changed = true;
            EJECT_speed = 100;
          }
          
          //send speed
          if (changed) {
            SendCumCommand(CUMSPEED, EJECT_speed, EJECT_ID);
          }
        
        
        }else if (lv_slider_get_value(ui_EJECTSPEEDslider) != speed){ //if slider moved
            EJECT_speed = lv_slider_get_value(ui_EJECTSPEEDslider);
            SendCumCommand(CUMSPEED, EJECT_speed, EJECT_ID);
        }
        char EJECT_speed_v[7];
        dtostrf(EJECT_speed, 6, 0, EJECT_speed_v);
        lv_label_set_text(ui_EJECTSPEEDvalue, EJECT_speed_v);

        //
        // Encoder 2 EJECT TIME
        //
        if(lv_slider_is_dragged(ui_EJECTTIMEslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_EJECTTIMEslider, EJECT_time, LV_ANIM_OFF);

		      if (encoder2.getCount() >= 2){      //EJECT_time up
            changed = true;
            EJECT_time += rampValue;
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
		      }else if (encoder2.getCount() <= -2){      //EJECT_time down
            changed = true;
            EJECT_time -=rampValue;
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
          }

          //EJECT_time min-max bounds
          if (EJECT_time < 0){
            changed = true;
            EJECT_time = 0;
          }
          if (EJECT_time > 61){
            changed = true;
            EJECT_time = 61;
          }
          
          //send EJECT_time
          if (changed) {
            SendCumCommand(CUMTIME, EJECT_time, EJECT_ID);
          }
        }else if(lv_slider_get_value(ui_EJECTTIMEslider) != EJECT_time){
            EJECT_time = lv_slider_get_value(ui_EJECTTIMEslider);
            SendCumCommand(CUMTIME, EJECT_time, EJECT_ID);
        }
        char EJECT_time_v[7];
        dtostrf(EJECT_time, 6, 0, EJECT_time_v);
        if (EJECT_time > 60) {
          lv_label_set_text(ui_EJECTTIMEvalue, T_CUM_CONSTANT);
        } else {
          lv_label_set_text(ui_EJECTTIMEvalue, EJECT_time_v);
        }
        
        //
        // Encoder 3 EJECT_size
        //
        if(lv_slider_is_dragged(ui_EJECTSIZEslider) == false){ //if knob gets rotated
          changed = false;

          lv_slider_set_value(ui_EJECTSIZEslider, EJECT_size, LV_ANIM_OFF);


		      if (encoder3.getCount() >= 2){      //EJECT_sizeup
            changed = true;
            EJECT_size+= rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
		      }else if (encoder3.getCount() <= -2){      //EJECT_sizedown
            changed = true;
            EJECT_size-= rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
          }

          //Stoke min-max bounds
          if (EJECT_size< 0){
            changed = true;
            EJECT_size= 0;
          }
          if (EJECT_size> 20){
            changed = true;
            EJECT_size= 20;
          }
          
          //send EJECT_size
          if (changed) {
            SendCumCommand(CUMSIZE, EJECT_size, EJECT_ID);
          }
        }else if(lv_slider_get_value(ui_EJECTSIZEslider) != EJECT_size){
            EJECT_size= lv_slider_get_value(ui_EJECTSIZEslider);
            SendCumCommand(CUMSIZE, EJECT_size, EJECT_ID);
        }

        char EJECT_size_v[7];
        dtostrf(EJECT_size, 6, 0, EJECT_size_v);
        lv_label_set_text(ui_EJECTSIZEvalue, EJECT_size_v);  //was lv_label_set_text(ui_EJECTSIZEvalue, stroke_v);

        //
        // Encoder4 EJECT_accel
        //
        if(lv_slider_is_dragged(ui_EJECTACCELslider) == false){
          changed = false;
          lv_slider_set_value(ui_EJECTACCELslider, EJECT_accel, LV_ANIM_OFF);

		      if (encoder4.getCount() >= 2){      // up
            changed = true;
            EJECT_accel += 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
		      }else if (encoder4.getCount() <= -2){      // down
            changed = true;
            EJECT_accel -= 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
            
          }

          //Stoke min-max bounds
          // clamp to slider range 0..max value of slider
          if (EJECT_accel < 0){
            changed = true;
            EJECT_accel = 0;
          }
          if (EJECT_accel > lv_slider_get_max_value(ui_EJECTACCELslider)){
            changed = true;
            EJECT_accel = lv_slider_get_max_value(ui_EJECTACCELslider);
          }

          if (changed) {
            SendCumCommand(CUMACCEL, EJECT_accel, EJECT_ID);
          }          
        } else if(lv_slider_get_value(ui_EJECTACCELslider) != EJECT_accel){
            EJECT_accel = lv_slider_get_value(ui_EJECTACCELslider);
            SendCumCommand(CUMACCEL, EJECT_accel, EJECT_ID);
        }

        // Check each button independently - not else-if chain to avoid blocking MX button
        if(click2_short_waspressed == true){
         lv_obj_send_event(ui_EJECTButtonL, LV_EVENT_SHORT_CLICKED, NULL);
          LogDebugPRIO("EJECT Short Left");
        }
        if(mxclick_short_waspressed == true){
         lv_obj_send_event(ui_EJECTButtonM, LV_EVENT_SHORT_CLICKED, NULL);
          LogDebugPRIO("EJECT Short Middle");
        }
        if(click3_short_waspressed == true){
         lv_obj_send_event(ui_EJECTButtonR, LV_EVENT_SHORT_CLICKED, NULL);
          LogDebugPRIO("EJECT Short Right");
        }
        if(click3_long_waspressed == true){
          EJECT_accel = 0;        //reset EJECT_accel to zero
          SendCommand(EJECT_accel, EJECT_accel, OSSM_ID);
          LogDebugPRIO("EJECT Long Right");
        }else if(click3_double_waspressed == true){
// what to do when double pressed press right button
        }
      }
      break;
      case ST_UI_Fist_IT_Settings: //Menu with FIST-IT control sliders
      {
        // TRACK CHANGES: Detect left/right encoder rotation once per loop
//        bool rotationDetectedLeft = (encoder2.getCount() >= 2 || encoder2.getCount() <= -2);
//        bool rotationDetectedRight = (encoder4.getCount() >= 2 || encoder4.getCount() <= -2);

        if(lv_obj_has_state(ui_lefty, LV_STATE_CHECKED) == 1){
          touch_disabled = true;
        }

          //RampHelper
          nowMs = millis();
          if (nowMs - rampMs <= rampTime && rampEnabled == true){ 
              if (rampValue <= maxRamp && encId == activeEncId){
            				++rampValue;
              }
          }else{
          rampValue = 1;
          activeEncId = encId;
          }

        //
        // Encoder 1 Fist_IT_speed 
        //
        if(lv_slider_is_dragged(ui_Fist_IT_SPEEDslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_Fist_IT_SPEEDslider, Fist_IT_speed, LV_ANIM_OFF);

          if (encoder1.getCount() >= 2){      //Fist_IT_speed up
            changed = true;
            Fist_IT_speed += rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
		      }else if (encoder1.getCount() <= -2){      //Fist_IT_speed down
            changed = true;
            Fist_IT_speed -=rampValue;
            encoder1.setCount(0);
            rampMs = millis();
            encId = 1;
          }

          //Fist_IT_speed min-max bounds
          if (Fist_IT_speed < 0){
            changed = true;
            Fist_IT_speed = 0;
          }
          if (Fist_IT_speed > 100){
            changed = true;
            Fist_IT_speed = 100;
          }
          
          //send speed
          if (changed) {
            SendFistCommand(Fist_IT_SPEED, Fist_IT_speed, FIST_ID);
          }
        
        
        }else if (lv_slider_get_value(ui_Fist_IT_SPEEDslider) != speed){ //if slider moved
            Fist_IT_speed = lv_slider_get_value(ui_Fist_IT_SPEEDslider);
            SendFistCommand(Fist_IT_SPEED, Fist_IT_speed, FIST_ID);
        }
        char Fist_IT_speed_v[7];
        dtostrf(Fist_IT_speed, 6, 0, Fist_IT_speed_v);
        lv_label_set_text(ui_Fist_IT_SPEEDvalue, Fist_IT_speed_v);

        //
        // Encoder 2 FIST-IT Rotation
        //
        if(lv_slider_is_dragged(ui_Fist_IT_ROTATIONslider) == false){ //if knob gets rotated
          changed = false;
          lv_slider_set_value(ui_Fist_IT_ROTATIONslider, Fist_IT_rotation, LV_ANIM_OFF);

		      if (encoder2.getCount() >= 2){      //Fist_IT_pause up
            changed = true;
            Fist_IT_rotation += rampValue;
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
		      }else if (encoder2.getCount() <= -2){      //Fist_IT_pause down
            changed = true;
            Fist_IT_rotation -=rampValue;
            encoder2.setCount(0);
            rampMs = millis();
            encId = 2;
          }

          //Fist_IT_rotation min-max bounds
          if (Fist_IT_rotation < 0){
            changed = true;
            Fist_IT_rotation = 0;
          }
          if (Fist_IT_rotation > 360){
            changed = true;
            Fist_IT_rotation = 360;
          }
          
          //send Fist_IT_rotation
          if (changed) {
            SendFistCommand(Fist_IT_ROTATION, Fist_IT_rotation, FIST_ID);
          }
        }else if(lv_slider_get_value(ui_Fist_IT_ROTATIONslider) != Fist_IT_pause){
            Fist_IT_rotation = lv_slider_get_value(ui_Fist_IT_ROTATIONslider);
            SendFistCommand(Fist_IT_ROTATION, Fist_IT_rotation, FIST_ID);
        }
        char Fist_IT_rotation_v[7];
        dtostrf(Fist_IT_rotation, 6, 0, Fist_IT_rotation_v);
        lv_label_set_text(ui_Fist_IT_ROTATIONvalue, Fist_IT_rotation_v);
        
        //
        // Encoder 3 FIST_IT_pause
        //
        if(lv_slider_is_dragged(ui_Fist_IT_PAUSEslider) == false){ //if knob gets rotated
          changed = false;

          lv_slider_set_value(ui_Fist_IT_PAUSEslider, Fist_IT_pause, LV_ANIM_OFF);


		      if (encoder3.getCount() >= 2){      //FIST_IT_pause up
            changed = true;
            Fist_IT_pause+= rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
		      }else if (encoder3.getCount() <= -2){      //FIST_IT_pause down
            changed = true;
            Fist_IT_pause-= rampValue;
            encoder3.setCount(0);
            rampMs = millis();
            encId = 3;
          }

          //Pause min-max bounds
          if (Fist_IT_pause< 0){
            changed = true;
            Fist_IT_pause= 0;
          }
          if (Fist_IT_pause> 100){
            changed = true;
            Fist_IT_pause= 100;
          }
          
          //send FIST_IT_pause
          if (changed) {
            SendFistCommand(Fist_IT_PAUSE, Fist_IT_pause, FIST_ID);
          }
        }else if(lv_slider_get_value(ui_Fist_IT_PAUSEslider) != Fist_IT_pause){
            Fist_IT_pause= lv_slider_get_value(ui_Fist_IT_PAUSEslider);
            SendFistCommand(Fist_IT_PAUSE, Fist_IT_pause, FIST_ID);
        }

        char Fist_IT_pause_v[7];
        dtostrf(Fist_IT_pause, 6, 0, Fist_IT_pause_v);
        lv_label_set_text(ui_Fist_IT_PAUSEvalue, Fist_IT_pause_v);  //was lv_label_set_text(ui_FIST-ITSIZEvalue, stroke_v);

        //
        // Encoder4 FIST_IT_accel
        //
        if(lv_slider_is_dragged(ui_Fist_IT_ACCELslider) == false){
          changed = false;
          lv_slider_set_value(ui_Fist_IT_ACCELslider, Fist_IT_accel, LV_ANIM_OFF);

		      if (encoder4.getCount() >= 2){      // up
            changed = true;
            Fist_IT_accel += 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
		      }else if (encoder4.getCount() <= -2){      // down
            changed = true;
            Fist_IT_accel -= 2;
            encoder4.setCount(0);
            rampMs = millis();
            encId = 4;
            
          }

          //Stoke min-max bounds
          if (Fist_IT_accel < 0){
            changed = true;
            Fist_IT_accel = 0;
          }
          if (Fist_IT_accel > 100){
            changed = true;
            Fist_IT_accel = 100;
          }

          if (changed) {
            SendFistCommand(Fist_IT_ACCEL, Fist_IT_accel, FIST_ID);
          }          
        } else if(lv_slider_get_value(ui_Fist_IT_ACCELslider) != Fist_IT_accel){
            Fist_IT_accel = lv_slider_get_value(ui_Fist_IT_ACCELslider);
            SendFistCommand(Fist_IT_ACCEL, Fist_IT_accel, FIST_ID);
        }

        // TRACK CHANGES: Only handle button presses if no rotation detected for that encoder
        if(click2_short_waspressed == true){
          LogDebugPRIO("FIST_IT Short Left");
         lv_obj_send_event(ui_Fist_IT_ButtonL, LV_EVENT_SHORT_CLICKED, NULL);
        }
        if(mxclick_short_waspressed == true){
          LogDebugPRIO("FIST_IT Short Middle"); 
         lv_obj_send_event(ui_Fist_IT_ButtonM, LV_EVENT_SHORT_CLICKED, NULL);
        }
        if(click3_short_waspressed == true){
          LogDebugPRIO("FIST_IT Short Right");
         lv_obj_send_event(ui_Fist_IT_ButtonR, LV_EVENT_SHORT_CLICKED, NULL);
        }
        if(click3_long_waspressed == true){
          LogDebugPRIO("FIST_IT Long Right");
          //Fist_IT_ACCEL = 0;        //reset FIST_IT_accel to zero
          SendFistCommand(Fist_IT_ACCEL, Fist_IT_accel, FIST_ID);
        }
        if(click3_double_waspressed == true){
          LogDebugPRIO("FIST_IT Double Right");

        }
      }
      break;



  }



     mxclick_long_waspressed = false;
     mxclick_short_waspressed = false;
     click2_short_waspressed = false;
     click2_long_waspressed = false;
     click2_double_waspressed = false;
     click3_short_waspressed = false;
     click3_long_waspressed = false;
     click3_double_waspressed = false;

  delay(5);
}

void CheckAllPeers(){
  esp_now_peer_num_t pn;
  esp_now_get_peer_num(&pn);
  LogDebug("Total Peer Count: ");
  LogDebug(pn.total_num);
  if (esp_now_is_peer_exist(OSSM_Address)){
    LogDebugFormatted("OSSM in peerlist with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
  }
  else{
    LogDebug("OSSM not in list");
//    ConnectToOSSM(OSSM_Address);
  }
    
  if (esp_now_is_peer_exist(EJECT_Address)){
    LogDebugFormatted("EJECT in peerlist with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
  }
  else{
    LogDebug("EJECT not in list");
    // Add the new peer
    memcpy(peerInfo.peer_addr, EJECT_Address, 6);

    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result == ESP_OK) {
      LogDebugFormatted("EJECT has been re-added to the list with MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
      Eject_paired = true;
    }
    else {
      LogDebug("Failed to add new peer");
    }
//    ConnectToEject(EJECT_Address);
  }
    
  if (esp_now_is_peer_exist(FIST_IT_Address)){
    LogDebugFormatted("FIST_IT in peerlist with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
  }
  else{
    LogDebug("FIST_IT not in list");
    // Add the new peer
    memcpy(peerInfo.peer_addr, FIST_IT_Address, 6);

    esp_err_t result = esp_now_add_peer(&peerInfo);
    if (result == ESP_OK) {
      LogDebugFormatted("FIST_IT has been re-added to the list with MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", FIST_IT_Address[0], FIST_IT_Address[1], FIST_IT_Address[2], FIST_IT_Address[3], FIST_IT_Address[4], FIST_IT_Address[5]);
      Fist_IT_paired = true;
    }
    else {
      LogDebug("Failed to add new peer");
    }
//    ConnectToEject(EJECT_Address);
  }
  if (esp_now_is_peer_exist(DEFAULT_Address)){
    LogDebugFormatted("Default address also in peerlist with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", DEFAULT_Address[0], DEFAULT_Address[1], DEFAULT_Address[2], DEFAULT_Address[3], DEFAULT_Address[4], DEFAULT_Address[5]);
  }
  else{
    LogDebug("Default address not in list anymore which is good");
//    ConnectToEject(EJECT_Address);
  }
}

void espNowRemoteTask(void *pvParameters)
{
  for(;;){
      //CheckAllPeers;
      if(Ossm_paired){
      LogDebug("Heartbeat OSSM");
//      LogDebugFormatted("OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = M5_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = OSSM_ID;
      esp_err_t result = esp_now_send(OSSM_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      if (result == ESP_OK) {
//        LogDebug("Sent ok");
      } 
      else {
        LogDebug("failed to send heartbeat");
      }
    }
    vTaskDelay(200/portTICK_PERIOD_MS);  // Use vTaskDelay instead of delay() to allow other tasks to run
      if(Eject_paired){
      LogDebug("Heartbeat EJECT");
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = M5_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = EJECT_ID;
      esp_err_t result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      //StatusMessageOut();
    }
    
  vTaskDelay(200/portTICK_PERIOD_MS);  // Use vTaskDelay instead of delay() to allow other tasks to run
    if(Fist_IT_paired){
      LogDebug("Heartbeat Fist_IT");
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = M5_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = FIST_ID;
      esp_err_t result = esp_now_send(FIST_IT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      //StatusMessageOut();
    }

    /*
    vTaskDelay(200/portTICK_PERIOD_MS);
    LogDebug("Heartbeat ALL");
    outgoingcontrol.esp_command = HEARTBEAT;
    outgoingcontrol.esp_sender = M5_ID;
    outgoingcontrol.esp_heartbeat = true;
    esp_err_t result = esp_now_send(DEFAULT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    //StatusMessageOut();
*/
    //problem when adding if eject not paired
    vTaskDelay(HEARTBEAT_INTERVAL);
  }
}

void espNowRemoteTaskEject(void *pvParameters)
{
  /*
  for(;;){
  esp_now_peer_num_t pn;
  esp_now_get_peer_num(&pn);
  LogDebug("Total Peer Count: ");
  LogDebug(pn.total_num);
  
    if(Eject_paired){
      LogDebug("Heartbeat EJECT");
      LogDebugFormatted("EJECT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = M5_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = EJECT_ID;
      esp_err_t result = esp_now_send(EJECT_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      if (result == ESP_OK) {
        LogDebug("Sent ok");
      } 
      else {
        LogDebug("failed to send");
      }

      //StatusMessageOut();
    }

    //problem when adding if eject not paired
    vTaskDelay(HEARTBEAT_INTERVAL);
  }
    */
  }

void vibrate(){
    if(vibrate_mode == true){
    M5.Power.setVibration(255);
    //M5.Power.Axp192.setLDO3(true);
    //M5.Power.Axp192.setLDO3(false);
    M5.Power.setVibration(0);
    }
}

void mxclick() {
  vibrate(225,75);
  mxclick_short_waspressed = true;
  LogDebugPRIO("MX Clicked");
} 

void mxlong(){
  vibrate(225,75);
  mxclick_long_waspressed = true;
  LogDebugPRIO("MX Long Clicked");
} 

void click2() {
  vibrate(225,75);
  click2_short_waspressed = true;
  LogDebugPRIO("Left Clicked");
} // click1

void c2long() {
  vibrate(225,75);
  click2_long_waspressed = true;
LogDebugPRIO("Left Long Clicked"); 
}

void c2double() {
  vibrate(225,75);
  click2_double_waspressed = true;
LogDebugPRIO("Left double Clicked"); 
}

void click3() {
  vibrate(225,75);
  click3_short_waspressed = true;
LogDebugPRIO("Right Clicked");
} // click1


void c3long() {
  vibrate(225,75);
  click3_long_waspressed = true;
  LogDebugPRIO("Right Long Clicked"); 
}

void c3double() {
  vibrate(225,75);
  click3_double_waspressed = true;
  LogDebugFormattedPRIO("Right double Clicked");
}

void StatusMessage(const uint8_t * mac){
///*
if (!incomingcontrol.esp_command==99){
LogDebug("Received message ");  
LogDebugFormatted("from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
LogDebugFormatted("OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
LogDebugFormatted("EJECT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);
LogDebug(status_message_text);
LogDebug("OSSM Paired: ");
LogDebug(Ossm_paired);
LogDebug("EJECT Paired: ");
LogDebug(Eject_paired);
LogDebug("Target: ");
LogDebug(incomingcontrol.esp_target);
LogDebug("Sender: ");
LogDebug(incomingcontrol.esp_sender);
LogDebug("Command: ");
LogDebug(incomingcontrol.esp_command);
LogDebug("Value: ");
LogDebug(incomingcontrol.esp_value);
LogDebug("");
//*/
}
}

void StatusMessageOut(){
/*  
LogDebug("===============Sending message===============")  ;
LogDebugFormatted("OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
LogDebugFormatted("EJECT addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", EJECT_Address[0], EJECT_Address[1], EJECT_Address[2], EJECT_Address[3], EJECT_Address[4], EJECT_Address[5]);

LogDebug("OSSM Paired: ");
LogDebug(Ossm_paired);
LogDebug("EJECT Paired: ");
LogDebug(Eject_paired);
LogDebug("Target: ");
LogDebug(outgoingcontrol.esp_target);
LogDebug("Sender: ");
LogDebug(outgoingcontrol.esp_sender);
LogDebug("Command: ");
LogDebug(outgoingcontrol.esp_command);
LogDebug("Value: ");
LogDebug(outgoingcontrol.esp_value);
LogDebug("");
*/
}

void ConnectToOSSM(const uint8_t * mac){

  
}

void ConnectToEject(const uint8_t * mac){

}

void ConnectToFist_IT(const uint8_t * mac){

}
