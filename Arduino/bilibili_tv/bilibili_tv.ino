// b站API：https://www.cnblogs.com/lovelymouse/p/7101212.html
// b站API: https://github.com/Passkou/bilibili_api
// ntp时钟： https://blog.csdn.net/qq_17351161/article/details/90551624?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task


// OPTIONAL: Assign default values here.
char wifiSSID[32] = ""; // Leave unset for wireless autoconfig. Note that these values will be lost
char wifiPass[64] = ""; // when updating, but that's probably OK because they will be saved in EEPROM.

////////////////////////////////////////////////////////////////////////////////////////////////////
// These defaults may be overwritten with values saved by the web interface
char haspNode[16] = "bilibili_tv";
char groupName[16] = "bilibili";
char configUser[32] = "admin";
char configPassword[32] = "";
char motionPinConfig[3] = "0";
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <FS.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <NTPClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750FVI.h>
#include <Adafruit_NeoPixel.h>

byte nextionReturnBuffer[128];                      // Byte array to pass around data coming from the panel
uint8_t nextionReturnIndex = 0;                     // Index for nextionReturnBuffer
uint8_t nextionActivePage = 0;                      // Track active LCD page
bool lcdConnected = false;                          // Set to true when we've heard something from the LCD
char wifiConfigPass[9];                             // AP config password, always 8 chars + NUL
char wifiConfigAP[19];                              // AP config SSID, haspNode + 3 chars
bool shouldSaveConfig = false;                      // Flag to save json config to SPIFFS
bool nextionReportPage0 = false;                    // If false, don't report page 0 sendme
unsigned long updateCheckTimer = 0;                 // Timer for update check
const unsigned long nextionCheckInterval = 5000;    // Time in msec between nextion connection checks
unsigned long nextionCheckTimer = 0;                // Timer for nextion connection checks
unsigned int nextionRetryMax = 5;                   // Attempt to connect to panel this many times
bool debugSerialEnabled = true;                     // Enable USB serial debug output
bool debugTelnetEnabled = false;                    // Enable telnet debug output
bool debugSerialD8Enabled = true;                   // Enable hardware serial debug output on pin D8
const unsigned long telnetInputMax = 128;           // Size of user input buffer for user telnet session
bool mdnsEnabled = true;                            // mDNS enabled
bool startupCompleteFlag = false;                   // Startup process has completed
const unsigned long connectTimeout = 300;           // Timeout for WiFi connection attempts in seconds
const unsigned long reConnectTimeout = 15;          // Timeout for WiFi reconnection attempts in seconds
byte espMac[6];                                     // Byte array to store our MAC address
String nextionModel;                                // Record reported model number of LCD panel
const byte nextionSuffix[] = {0xFF, 0xFF, 0xFF};    // Standard suffix for Nextion commands

uint8_t LeftLedPin = D6;                       // Pin for Bilibili_TV left switch (GPIO5/D3)
uint8_t RightLedPin = D5;                       // Pin for Bilibili_TV right switch (GPIO4/D5)
uint8_t LightStripPin = D8;                      //ws2812灯带Din
uint8_t SpeakPin = D0;                              //蜂鸣器
bool motionEnabled = false;                         // Motion sensor is enabled
uint8_t motionPin = D3;                              // GPIO input pin for motion sensor if connected and enabled
bool motionActive = false;                          // Motion is being detected
const unsigned long oldfans = 0;

const unsigned long fansUpdateInterval = 10;     // 粉丝数据刷新间隔
const unsigned long videoUpdateInterval = 5000;    // 视频数据刷新间隔
const unsigned long spaceUpdateInterval = 8;    // 个人中心刷新间隔
const unsigned long envUpdateInterval = 5000;      // bme280刷新间隔
const unsigned long luxUpdateInterval = 5000;      // bh1750刷新间隔
const unsigned long ntpUpdateInterval = 5;      // ntp刷新间隔

unsigned long fansUpdateTimer = 0;                 // 粉丝数据刷新计数
unsigned long videoUpdateTimer = 0;                // 视频数据刷新计数
unsigned long spaceUpdateTimer = 0;                // 个人中心刷新计数
unsigned long envUpdateTimer = 0;                  // bme280刷新计数
unsigned long luxUpdateTimer = 0;                  // bh1750刷新计数
unsigned long ntpUpdateTimer = 0;                  // ntp刷新计数
unsigned long yesterdayFans = 0;                   // 昨日粉丝量
unsigned long yesterdayPlay = 0;                   // 昨日播放量
unsigned long todayFans = 0;                       // 今日粉丝量
unsigned long todayPlay = 0;                       // 今日播放量
unsigned long Fans = 0;                       // 粉丝量
unsigned long Play = 0;                       // 播放量

const char* fans_api = "/x/relation/stat?vmid=";
const char* video_api = "/archive_stat/stat?aid=";
const char* space_api = "/x/space/upstat?mid=";
const char* host = "api.bilibili.com";
const char* uid = "uid";                   //xxx --你的b站uid
const char* aid = "aid";                   //xxx --视频aid
const unsigned long HTTP_TIMEOUT = 5000;   // 服务器最大响应时间
const size_t  MAX_CONTENT_SIZE  = 1000;    // HTTP最大响应大小
bool requestcount = 1;  //响应计数

// 粉丝和关注
struct FansData{
  long follower;//粉丝
  long following;//关注    
};
// 视频播放
struct VideoData{
  long view;//播放量
  long coin;//硬币
  long like;//点赞
  long favorite;//收藏
  long danmaku;//弹幕    
};
// 个人中心
struct SpaceData{
  long archive_view;//总播放量
  long article_view;//硬币
  long likes;//点赞   
};

Adafruit_BME280 bme; // I2C
BH1750FVI LightSensor(BH1750FVI::k_DevModeContLowRes);
ESP8266WebServer webServer(80);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient client;
char response[MAX_CONTENT_SIZE];
char endOfHeaders[] = "\r\n\r\n";
MDNSResponder::hMDNSService hMDNSService;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"ntp1.aliyun.com",60*60*8, 30*60*1000);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(21, LightStripPin, NEO_GRB + NEO_KHZ800);
// Additional CSS style to match Hass theme
const char HASP_STYLE[] = "<style>button{background-color:#03A9F4;}body{width:60%;margin:auto;}input:invalid{border:1px solid red;}input[type=checkbox]{width:20px;}</style>";


////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{ // System setup
  Serial.begin(115200);  // Serial - LCD RX (after swap), debug TX
  Serial1.begin(9600); // Serial1 - LCD TX, no RX
  Serial.swap();
  debugPrintln(String(F("SYSTEM: Last reset reason: ")) + String(ESP.getResetInfo()));
  while (!lcdConnected && (millis() < 5000))
  { // Wait up to 5 seconds for serial input from LCD
    nextionHandleInput();
  }
  if (lcdConnected)
  {
    debugPrintln(F("HMI: LCD responding, continuing program load"));
    nextionSendCmd("connect");
  }
  else
  {
    debugPrintln(F("HMI: LCD not responding, continuing program load"));
  }

  espWifiSetup(); // Start up networking
  webServer.on("/", webHandleRoot);
  webServer.on("/saveConfig", webHandleSaveConfig);
  webServer.on("/resetConfig", webHandleResetConfig);
  webServer.on("/reboot", webHandleReboot);
  webServer.onNotFound(webHandleNotFound);
  webServer.begin();
  debugPrintln(String(F("HTTP: Server started @ http://")) + WiFi.localIP().toString());
  debugPrintln(F("SYSTEM: System init complete."));
  telnetClient.setTimeout(HTTP_TIMEOUT);
  timeClient.begin();
  LightSensor.begin();  
  LedSetup();
  LightStripSetup();
  BreathLamp(2);
  SpeakSetup();
  enviromnentSetup();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{ // Main execution loop

  if (nextionHandleInput())
  { // Process user input from HMI
    nextionProcessInput();
  }
  
  if(fansUpdateTimer>=fansUpdateInterval)
  {
    BlinkLed();
    BreathLamp(2);
    RequestFansData();
    fansUpdateTimer= 0;
    }
    
  if(spaceUpdateTimer>=spaceUpdateInterval)
  {
    BlinkLed();
    BreathLamp(2);
    RequestSpaceData();
    spaceUpdateTimer= 0; 
    }
//  debugPrintln("spaceUpdateTimer = ") ;
//  debugPrintln(String(spaceUpdateTimer));
  if(ntpUpdateTimer<ntpUpdateInterval)
  {
    timeClient.update();
    debugPrintln(timeClient.getFormattedTime());
    if(timeClient.getHours()==23){
      debugPrintln("hours is 23");
      if(timeClient.getMinutes()==59){
        debugPrintln("Minutes is 59");
        yesterdayFans=Fans;
        yesterdayPlay=Play;
        }
      }
    ntpUpdateTimer= 0;
    }
  BatValueUpdate();
  LuxUpdate();
  fansUpdateTimer++;
  spaceUpdateTimer++;
  envUpdateTimer++;
  ntpUpdateTimer++;
  environmentUpdate();
//  LightStripShow();
//  SpeakSound();
 // 
  delay(1000); // Spooky voodoo which claims to improve WiFi stability.
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * LED初始化
 */
void LedSetup(){
  pinMode(LeftLedPin,OUTPUT);
  pinMode(RightLedPin,OUTPUT);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Blink LED
 */
void BlinkLed(){
    digitalWrite(LeftLedPin,HIGH);
    digitalWrite(RightLedPin,HIGH);
    delay(100);
    digitalWrite(LeftLedPin,LOW);
    digitalWrite(RightLedPin,LOW);
    delay(100);
    digitalWrite(LeftLedPin,HIGH);
    digitalWrite(RightLedPin,HIGH);
    delay(100);
    digitalWrite(LeftLedPin,LOW);
    digitalWrite(RightLedPin,LOW);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 蜂鸣器初始化
 */
void SpeakSetup(){
  pinMode(SpeakPin,OUTPUT);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 蜂鸣器
 */
void SpeakSound(){
    digitalWrite(SpeakPin,HIGH);
    delay(100);
    digitalWrite(SpeakPin,LOW);
    delay(100);
    digitalWrite(SpeakPin,HIGH);
    delay(100);
    digitalWrite(SpeakPin,LOW);
  }

///////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 闹钟
 */
void Alarm(){
    digitalWrite(SpeakPin,HIGH);
    delay(1000);
    digitalWrite(SpeakPin,LOW);
    delay(1000);
    digitalWrite(SpeakPin,HIGH);
    delay(1000);
    digitalWrite(SpeakPin,LOW);
  }
  
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 电量显示
 */
void BatValueUpdate(){
  int sensorValue1 = analogRead(A0);
  delay(100);
  int sensorValue2 = analogRead(A0);
  delay(100);
  int sensorValue3 = analogRead(A0);
  delay(100);
  int sensorValue = (sensorValue1+sensorValue2+sensorValue3)/3 ;
  int batValue = sensorValue*(16200+49400)/165888;
  debugPrintln("BatteryValue = ");
  debugPrintln(String(batValue));
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 光照度监测
 */
void LuxUpdate(){
  uint16_t lux = LightSensor.GetLightIntensity();
  debugPrintln("Light: ");
  debugPrintln(String(lux));
  delay(250);
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * 请求粉丝数据
 */
void RequestFansData(){
  //判断tcp client是否处于连接状态，不是就建立连接
  while (!client.connected()){
     if (!client.connect(host, 80)){
         debugPrintln("connection....");
         delay(500);
     }
  }
  //发送粉丝API请求
  if (sendRequest(fans_api, host, uid)&& skipResponseHeaders()){
      //清除缓存
      clrEsp8266ResponseBuffer();
      //读取响应数据
      readReponseContent(response, sizeof(response));
      FansData fansdata;
      if(parseFansData(response,&fansdata)){
        printFansData(&fansdata);
      }
   }
   stopConnect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * 请求视频数据
 */
void RequestVideoData(){
  //判断tcp client是否处于连接状态，不是就建立连接
  while (!client.connected()){
     if (!client.connect(host, 80)){
         debugPrintln("connection....");
         delay(500);
     }
  }
  //发送视频API请求
  if (sendRequest(video_api, host, aid)&& skipResponseHeaders()){
    //清除缓存
    clrEsp8266ResponseBuffer();
    //读取响应数据
    readReponseContent(response, sizeof(response));
    VideoData videodata;
    if(parseVideoData(response,&videodata)){
      printVideoData(&videodata);
    }
  }
  stopConnect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * 请求个人主页数据
 */
void RequestSpaceData(){
  //判断tcp client是否处于连接状态，不是就建立连接
  while (!client.connected()){
     if (!client.connect(host, 80)){
         debugPrintln("connection....");
         delay(500);
     }
  }
  //发送个人主页API请求
 // if(requestcount==0){
    if (sendRequest(space_api, host, uid)&& skipResponseHeaders()){
      //清除缓存
      clrEsp8266ResponseBuffer();
      //读取响应数据
      readReponseContent(response, sizeof(response));
      SpaceData spacedata;
      if(parseSpaceData(response,&spacedata)){
        printSpaceData(&spacedata);
      }
     }
   // }
    stopConnect();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* @发送http请求
*/
bool sendRequest(const char* api, const char* host, const char* id){
  client.print(String("GET ") + api + id + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connention: close\r\n\r\n");
  debugPrintln("create a request:");
  debugPrintln(String("GET ") + api + id + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connention: close\r\n");
  delay(1000);
  return true;   
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* @Desc 跳过 HTTP 头，使我们在响应正文的开头
*/
bool skipResponseHeaders() {
  // HTTP headers end with an empty line
  bool ok = client.find(endOfHeaders);
  if (!ok) {
    debugPrintln("No response or invalid response!");
  }
  return ok;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* @Desc 从HTTP服务器响应中读取正文
*/
void readReponseContent(char* content, size_t maxSize) {
  size_t length = client.peekBytes(content, maxSize);
  delay(1000);
  debugPrintln("Get the data from Internet!");
  content[length] = 0;
  debugPrintln(content);
  debugPrintln("Read data Over!");
  client.flush();//清除一下缓冲
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/**
* 复制粉丝数据
*/
bool parseFansData(char* content, struct FansData* fansdata) 
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, content);
  if (error)
  {
    debugPrintln("..");
    debugPrintln(error.c_str());
    return false;
    }
  //复制我们感兴趣的字符串
  fansdata->follower = doc["data"]["follower"];
  fansdata->following = doc["data"]["following"];
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 复制视频数据 
*/
bool parseVideoData(char* content, struct VideoData* videodata) 
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, content);
  if (error)
  {
    debugPrintln("..");
    debugPrintln(error.c_str());
    return false;
    }
  //复制我们感兴趣的字符串
  videodata->view = doc["data"]["view"];
  videodata->coin = doc["data"]["coin"];
  videodata->like = doc["data"]["like"];
  videodata->favorite = doc["data"]["favorite"];
  videodata->danmaku = doc["data"]["danmaku"];
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 复制个人主页数据
*/
bool parseSpaceData(char* content, struct SpaceData* spacedata) 
{
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, content);
  if (error)
  {
    debugPrintln("..");
    debugPrintln(error.c_str());
    return false;
    }
  //复制我们感兴趣的字符串
  spacedata->archive_view = doc["data"]["archive"]["view"];
  spacedata->article_view = doc["data"]["article"]["view"];
  spacedata->likes = doc["data"]["likes"];
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 打印粉丝数据
*/
void printFansData(const struct FansData* fansdata){
  debugPrintln("Print Fans data :");
  debugPrintln("Follower : ");
  
  nextionSetAttr("page1.t0.txt", "\""+String(fansdata->follower)+"\"");
  Fans = fansdata->follower;
  debugPrintln(String(fansdata->follower));
//  if(fansdata->follower!=oldfans){
//    BlinkLed();
//    }
  if(fansdata->follower>=yesterdayFans)
  {
    todayFans=fansdata->follower-yesterdayFans;
    nextionSetAttr("page3.t3.txt", "\""+String(todayFans)+"\"");
   }
  else
  {
    todayFans=yesterdayFans-fansdata->following;
    nextionSetAttr("page3.t3.txt", "\""+'-'+String(todayFans)+"\"");
    }
    
  debugPrintln("Following : ");
  debugPrintln(String(fansdata->following));
  debugPrintln("\r\n");  
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 打印视频数据
*/
void printVideoData(const struct VideoData* videodata){
  debugPrintln("Print Video data :");
  debugPrintln("view : ");
  debugPrintln(String(videodata->view));
  debugPrintln("coin : ");
  debugPrintln(String(videodata->coin));
  debugPrintln("like : ");
  debugPrintln(String(videodata->like));
  debugPrintln("favorite : ");
  debugPrintln(String(videodata->favorite));
  debugPrintln("danmaku : ");
  debugPrintln(String(videodata->danmaku));
  debugPrintln("\r\n");  
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 打印个人主页数据
*/
void printSpaceData(const struct SpaceData* spacedata){
  debugPrintln("Print Video data :");
  debugPrintln("archive_view : ");
  debugPrintln(String(spacedata->archive_view));
  
  nextionSetAttr("page1.t1.txt", "\""+String(spacedata->archive_view)+"\"");
  if(spacedata->archive_view>=yesterdayPlay)
  {
    todayPlay=spacedata->archive_view-yesterdayPlay;
    nextionSetAttr("page3.t4.txt", "\""+String(todayPlay)+"\"");
    }
  Play = spacedata->archive_view;
  debugPrintln("article_view : ");
  debugPrintln(String(spacedata->article_view));
  debugPrintln("likes : ");
  debugPrintln(String(spacedata->likes)); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 关闭与HTTP服务器连接
*/
void stopConnect(){
  debugPrintln("Disconnect");
  client.stop();
}

void clrEsp8266ResponseBuffer(void){
  memset(response, 0, MAX_CONTENT_SIZE);  //清空
}
////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ws2812灯带初始化
 */
void LightStripSetup(){
  strip.begin();
  strip.setBrightness(50);
  strip.show();
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * ws2812灯带显示
 */
void LightStripShow(){
  colorWipe(strip.Color(0, 0, 255), 50); // Blue
  colorWipe(strip.Color(0, 0, 0), 50); // Blue
  }
  
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * 灯效组
 */
// Fill the dots one after the other with a color

void BreathLamp(uint8_t wait) {
  for(uint16_t j=0; j<255; j++)
  {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, j));
      }
    strip.show();
    delay(wait);
    }
  for(uint16_t k=255; k>0; k--)
  {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(0, 0, k));
      }
    strip.show();
    delay(wait);
    }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
/*
* BME280初始化程序
*/
void enviromnentSetup()
{
  unsigned status;
  status = bme.begin();
  if (!status) {
    debugPrintln("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
}  

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* BME280更新程序
*/
void environmentUpdate()
{
  debugPrintln("Temperature = ");
  debugPrintln(String(bme.readTemperature()));  
  debugPrintln(" C"); 
  nextionSetAttr("page2.t9.txt", "\""+String(bme.readTemperature())+"C"+"\"");

  debugPrintln("Humidity = ");
  debugPrintln(String(bme.readHumidity()));  
  debugPrintln(" %"); 
  nextionSetAttr("page2.t10.txt", "\""+String(bme.readHumidity())+"%"+"\"");

  debugPrintln("Pressure = ");
  debugPrintln(String(bme.readPressure() / 100.0F));  
  debugPrintln(" hPa"); 
  nextionSetAttr("page2.t11.txt", "\""+String(bme.readPressure() / 100000.0F)+"Kpa"+"\"");
  }
  
//////////////////////////////////////////////////////////////////////////////////////
/*
* HMI输入处理
*/
bool nextionHandleInput()
{ // Handle incoming serial data from the Nextion panel
  // This will collect serial data from the panel and place it into the global buffer
  // nextionReturnBuffer[nextionReturnIndex]
  // Return: true if we've received a string of 3 consecutive 0xFF values
  // Return: false otherwise
  bool nextionCommandComplete = false;
  static int nextionTermByteCnt = 0;   // counter for our 3 consecutive 0xFFs
  static String hmiDebug = "HMI IN: "; // assemble a string for debug output

  if (Serial.available())
  {
    lcdConnected = true;
    byte nextionCommandByte = Serial.read();
    hmiDebug += (" 0x" + String(nextionCommandByte, HEX));
    // check to see if we have one of 3 consecutive 0xFF which indicates the end of a command
    if (nextionCommandByte == 0xFF)
    {
      nextionTermByteCnt++;
      if (nextionTermByteCnt >= 3)
      { // We have received a complete command
        nextionCommandComplete = true;
        nextionTermByteCnt = 0; // reset counter
      }
    }
    else
    {
      nextionTermByteCnt = 0; // reset counter if a non-term byte was encountered
    }
    nextionReturnBuffer[nextionReturnIndex] = nextionCommandByte;
    nextionReturnIndex++;
  }
  if (nextionCommandComplete)
  {
    debugPrintln(hmiDebug);
    hmiDebug = "HMI IN: ";
  }
  return nextionCommandComplete;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionProcessInput()
{ // Process incoming serial commands from the Nextion panel
  // Command reference: https://www.itead.cc/wiki/Nextion_Instruction_Set#Format_of_Device_Return_Data
  // tl;dr, command byte, command data, 0xFF 0xFF 0xFF

  if (nextionReturnBuffer[0] == 0x65)
  { // Handle incoming touch command
    // 0x65+Page ID+Component ID+TouchEvent+End
    // Return this data when the touch event created by the user is pressed.
    // Definition of TouchEvent: Press Event 0x01, Release Event 0X00
    // Example: 0x65 0x00 0x02 0x01 0xFF 0xFF 0xFF
    // Meaning: Touch Event, Page 0, Object 2, Press
    String nextionPage = String(nextionReturnBuffer[1]);
    String nextionButtonID = String(nextionReturnBuffer[2]);
    byte nextionButtonAction = nextionReturnBuffer[3];

    if (nextionButtonAction == 0x01)
    {
      debugPrintln(String(F("HMI IN: [Button ON] 'p[")) + nextionPage + "].b[" + nextionButtonID + "]'");
    }
    if (nextionButtonAction == 0x00)
    {
      debugPrintln(String(F("HMI IN: [Button OFF] 'p[")) + nextionPage + "].b[" + nextionButtonID + "]'");
      // Now see if this object has a .val that might have been updated.  Works for sliders,
      // two-state buttons, etc, throws a 0x1A error for normal buttons which we'll catch and ignore
      nextionGetAttr("p[" + nextionPage + "].b[" + nextionButtonID + "].val");
    }
  }
  else if (nextionReturnBuffer[0] == 0x66)
  { // Handle incoming "sendme" page number
    // 0x66+PageNum+End
    // Example: 0x66 0x02 0xFF 0xFF 0xFF
    // Meaning: page 2
    String nextionPage = String(nextionReturnBuffer[1]);
    debugPrintln(String(F("HMI IN: [sendme Page] '")) + nextionPage + "'");
    if ((nextionActivePage != nextionPage.toInt()) && ((nextionPage != "0") || nextionReportPage0))
    { // If we have a new page AND ( (it's not "0") OR (we've set the flag to report 0 anyway) )
      nextionActivePage = nextionPage.toInt();
    }
  }
  else if (nextionReturnBuffer[0] == 0x67)
  { // Handle touch coordinate data
    // 0X67+Coordinate X High+Coordinate X Low+Coordinate Y High+Coordinate Y Low+TouchEvent+End
    // Example: 0X67 0X00 0X7A 0X00 0X1E 0X01 0XFF 0XFF 0XFF
    // Meaning: Coordinate (122,30), Touch Event: Press
    // issue Nextion command "sendxy=1" to enable this output
    uint16_t xCoord = nextionReturnBuffer[1];
    xCoord = xCoord * 256 + nextionReturnBuffer[2];
    uint16_t yCoord = nextionReturnBuffer[3];
    yCoord = yCoord * 256 + nextionReturnBuffer[4];
    String xyCoord = String(xCoord) + ',' + String(yCoord);
    byte nextionTouchAction = nextionReturnBuffer[5];
    if (nextionTouchAction == 0x01)
    {
      debugPrintln(String(F("HMI IN: [Touch ON] '")) + xyCoord + "'");
    }
    else if (nextionTouchAction == 0x00)
    {
      debugPrintln(String(F("HMI IN: [Touch OFF] '")) + xyCoord + "'");
    }
  }
  else if (nextionReturnBuffer[0] == 0x70)
  { // Handle get string return
    // 0x70+ASCII string+End
    // Example: 0x70 0x41 0x42 0x43 0x44 0x31 0x32 0x33 0x34 0xFF 0xFF 0xFF
    // Meaning: String data, ABCD1234
    String getString;
    for (int i = 1; i < nextionReturnIndex - 3; i++)
    { // convert the payload into a string
      getString += (char)nextionReturnBuffer[i];
    }
    debugPrintln(String(F("HMI IN: [String Return] '")) + getString + "'");
  }
  else if (nextionReturnBuffer[0] == 0x71)
  { // Handle get int return
    // 0x71+byte1+byte2+byte3+byte4+End (4 byte little endian)
    // Example: 0x71 0x7B 0x00 0x00 0x00 0xFF 0xFF 0xFF
    // Meaning: Integer data, 123
    unsigned long getInt = nextionReturnBuffer[4];
    getInt = getInt * 256 + nextionReturnBuffer[3];
    getInt = getInt * 256 + nextionReturnBuffer[2];
    getInt = getInt * 256 + nextionReturnBuffer[1];
    String getString = String(getInt);
    debugPrintln(String(F("HMI IN: [Int Return] '")) + getString + "'");


  }
  else if (nextionReturnBuffer[0] == 0x63 && nextionReturnBuffer[1] == 0x6f && nextionReturnBuffer[2] == 0x6d && nextionReturnBuffer[3] == 0x6f && nextionReturnBuffer[4] == 0x6b)
  { // Catch 'comok' response to 'connect' command: https://www.itead.cc/blog/nextion-hmi-upload-protocol
    String comokField;
    uint8_t comokFieldCount = 0;
    byte comokFieldSeperator = 0x2c; // ","

    for (uint8_t i = 0; i <= nextionReturnIndex; i++)
    { // cycle through each byte looking for our field seperator
      if (nextionReturnBuffer[i] == comokFieldSeperator)
      { // Found the end of a field, so do something with it.  Maybe.
        if (comokFieldCount == 2)
        {
          nextionModel = comokField;
          debugPrintln(String(F("HMI IN: nextionModel: ")) + nextionModel);
        }
        comokFieldCount++;
        comokField = "";
      }
      else
      {
        comokField += String(char(nextionReturnBuffer[i]));
      }
    }
  }

  else if (nextionReturnBuffer[0] == 0x1A)
  { // Catch 0x1A error, possibly from .val query against things that might not support that request
    // 0x1A+End
    // ERROR: Variable name invalid
    // We'll be triggering this a lot due to requesting .val on every component that sends us a Touch Off
  }
  nextionReturnIndex = 0; // Done handling the buffer, reset index back to 0
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionSetAttr(String hmiAttribute, String hmiValue)
{ // Set the value of a Nextion component attribute
  Serial1.print(hmiAttribute);
  Serial1.print("=");
  Serial1.print(utf8ascii(hmiValue));
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  debugPrintln(String(F("HMI OUT: '")) + hmiAttribute + "=" + hmiValue + "'");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionGetAttr(String hmiAttribute)
{ // Get the value of a Nextion component attribute
  // This will only send the command to the panel requesting the attribute, the actual
  Serial1.print("get " + hmiAttribute);
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  debugPrintln(String(F("HMI OUT: 'get ")) + hmiAttribute + "'");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionSendCmd(String nextionCmd)
{ // Send a raw command to the Nextion panel
  Serial1.print(utf8ascii(nextionCmd));
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  debugPrintln(String(F("HMI OUT: ")) + nextionCmd);
}


////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionConnect()
{
  if ((millis() - nextionCheckTimer) >= nextionCheckInterval)
  {
    static unsigned int nextionRetryCount = 0;
    if ((nextionModel.length() == 0) && (nextionRetryCount < (nextionRetryMax - 2)))
    { // Try issuing the "connect" command a few times
      debugPrintln(F("HMI: sending Nextion connect request"));
      nextionSendCmd("connect");
      nextionRetryCount++;
      nextionCheckTimer = millis();
    }
    else if ((nextionModel.length() == 0) && (nextionRetryCount < nextionRetryMax))
    { // If we still don't have model info, try to change nextion serial speed from 9600 to 115200
      nextionSetSpeed();
      nextionRetryCount++;
      debugPrintln(F("HMI: sending Nextion serial speed 115200 request"));
      nextionCheckTimer = millis();
    }
    else if (nextionRetryCount <= nextionRetryMax)
    {
      if (nextionModel.length() == 0)
      { // one last hail mary, maybe the serial speed is set correctly now
        nextionSendCmd("connect");
      }
      nextionCheckTimer = millis();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void nextionSetSpeed()
{
  debugPrintln(F("HMI: No Nextion response, attempting 9600bps connection"));
  Serial1.begin(9600);
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  Serial1.print("bauds=115200");
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  Serial1.flush();
  Serial1.begin(115200);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*void nextionReset()
{
  debugPrintln(F("HMI: Rebooting LCD"));
  digitalWrite(nextionResetPin, LOW);
  Serial1.print("rest");
  Serial1.write(nextionSuffix, sizeof(nextionSuffix));
  Serial1.flush();
  delay(100);
  digitalWrite(nextionResetPin, HIGH);

  unsigned long lcdResetTimer = millis();
  const unsigned long lcdResetTimeout = 5000;

  lcdConnected = false;
  while (!lcdConnected && (millis() < (lcdResetTimer + lcdResetTimeout)))
  {
    nextionHandleInput();
  }
  if (lcdConnected)
  {
    debugPrintln(F("HMI: Rebooting LCD completed"));
    if (nextionActivePage)
    {
      nextionSendCmd("page " + String(nextionActivePage));
    }
  }
  else
  {
    debugPrintln(F("ERROR: Rebooting LCD completed, but LCD is not responding."));
  }
}*/

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片WiFi初始化
*/
void espWifiSetup()
{ // Connect to WiFi
  nextionSendCmd("page 0");
  nextionSetAttr("p[0].b[1].txt", "\"WiFi Connecting\"");

  // Read our MAC address and save it to espMac
  WiFi.macAddress(espMac);
  // Assign our hostname before connecting to WiFi
  WiFi.hostname(haspNode);
  // Tell WiFi to autoreconnect if connection has dropped
  WiFi.setAutoReconnect(true);

  if (String(wifiSSID) == "")
  { // If the sketch has no defined a static wifiSSID to connect to,
    // use WiFiManager to collect required information from the user.

    // id/name, placeholder/prompt, default value, length, extra tags
    WiFiManagerParameter custom_haspNodeHeader("<br/><br/><b>HASP Node Name</b>");
    WiFiManagerParameter custom_haspNode("haspNode", "HASP Node (required. lowercase letters, numbers, and _ only)", haspNode, 15, " maxlength=15 required pattern='[a-z0-9_]*'");
    WiFiManagerParameter custom_groupName("groupName", "Group Name (required)", groupName, 15, " maxlength=15 required");
    WiFiManagerParameter custom_configHeader("<br/><br/><b>Admin access</b>");
    WiFiManagerParameter custom_configUser("configUser", "Config User", configUser, 15, " maxlength=31'");
    WiFiManagerParameter custom_configPassword("configPassword", "Config Password", configPassword, 31, " maxlength=31 type='password'");

    // WiFiManager local initialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    wifiManager.setSaveConfigCallback(configSaveCallback); // set config save notify callback
    wifiManager.setCustomHeadElement(HASP_STYLE);          // add custom style

    // Add all your parameters here
    wifiManager.addParameter(&custom_haspNodeHeader);
    wifiManager.addParameter(&custom_haspNode);
    wifiManager.addParameter(&custom_groupName);
    wifiManager.addParameter(&custom_configHeader);
    wifiManager.addParameter(&custom_configUser);
    wifiManager.addParameter(&custom_configPassword);

    // Timeout config portal after connectTimeout seconds, useful if configured wifi network was temporarily unavailable
    wifiManager.setTimeout(connectTimeout);

    wifiManager.setAPCallback(espWifiConfigCallback);

    // Construct AP name
    char espMac5[1];
    sprintf(espMac5, "%02x", espMac[5]);
    String strWifiConfigAP = String(haspNode).substring(0, 9) + "-" + String(espMac5);
    strWifiConfigAP.toCharArray(wifiConfigAP, (strWifiConfigAP.length() + 1));
    // Construct a WiFi SSID password using bytes [3] and [4] of our MAC
    char espMac34[2];
    sprintf(espMac34, "%02x%02x", espMac[3], espMac[4]);
    String strConfigPass = "biliconfig";
    strConfigPass.toCharArray(wifiConfigPass, 9);

    // Fetches SSID and pass from EEPROM and tries to connect
    // If it does not connect it starts an access point with the specified name
    // and goes into a blocking loop awaiting configuration.
    if (!wifiManager.autoConnect(wifiConfigAP, wifiConfigPass))
    { // Reset and try again
      debugPrintln(F("WIFI: Failed to connect and hit timeout"));
      espReset();
    }

    // Read updated parameters
    strcpy(haspNode, custom_haspNode.getValue());
    strcpy(groupName, custom_groupName.getValue());
    strcpy(configUser, custom_configUser.getValue());
    strcpy(configPassword, custom_configPassword.getValue());

    if (shouldSaveConfig)
    { // Save the custom parameters to FS
      configSave();
    }
  }
  else
  { // wifiSSID has been defined, so attempt to connect to it forever
    debugPrintln(String(F("Connecting to WiFi network: ")) + String(wifiSSID));
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID, wifiPass);

    unsigned long wifiReconnectTimer = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      if (millis() >= (wifiReconnectTimer + (connectTimeout * 1000)))
      { // If we've been trying to reconnect for connectTimeout seconds, reboot and try again
        debugPrintln(F("WIFI: Failed to connect and hit timeout"));
        espReset();
      }
    }
  }
  // If you get here you have connected to WiFi
  nextionSetAttr("p[0].b[1].txt", "\"WiFi Connected:\\r" + WiFi.localIP().toString() + "\"");
  debugPrintln(String(F("WIFI: Connected successfully and assigned IP: ")) + WiFi.localIP().toString());
  if (nextionActivePage)
  {
    nextionSendCmd("page " + String(nextionActivePage));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片WiFi重新连接
*/
void espWifiReconnect()
{ // Existing WiFi connection dropped, try to reconnect
  debugPrintln(F("Reconnecting to WiFi network..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPass);

  unsigned long wifiReconnectTimer = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (millis() >= (wifiReconnectTimer + (reConnectTimeout * 1000)))
    { // If we've been trying to reconnect for reConnectTimeout seconds, reboot and try again
      debugPrintln(F("WIFI: Failed to reconnect and hit timeout"));
      espReset();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片WiFi设置返回
*/
void espWifiConfigCallback(WiFiManager *myWiFiManager)
{ // Notify the user that we're entering config mode
  debugPrintln(F("WIFI: Failed to connect to assigned AP, entering config mode"));
  while (millis() < 800)
  { // for factory-reset system this will be called before display is responsive. give it a second.
    delay(10);
  }
  nextionSendCmd("page 0");
  nextionSetAttr("p[0].b[1].txt", "\"Configure HASP:\\rAP:" + String(wifiConfigAP) + "\\rPass:" + String(wifiConfigPass) + "\\r\\r\\r\\r\\r\\r\\rWeb:192.168.4.1\"");
  nextionSetAttr("p[0].b[3].txt", "\"WIFI:S:" + String(wifiConfigAP) + ";T:WPA;P:" + String(wifiConfigPass) + ";;\"");
  nextionSendCmd("vis 3,1");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片重置
*/
void espReset()
{
  debugPrintln(F("RESET: HASP reset"));
  ESP.reset();
  delay(5000);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片读取设置
*/
void configRead()
{ // Read saved config.json from SPIFFS
  debugPrintln(F("SPIFFS: mounting SPIFFS"));
  if (SPIFFS.begin())
  {
    if (SPIFFS.exists("/config.json"))
    { // File exists, reading and loading
      debugPrintln(F("SPIFFS: reading /config.json"));
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        size_t configFileSize = configFile.size(); // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize]);
        configFile.readBytes(buf.get(), configFileSize);

        DynamicJsonDocument configJson(1024);
        DeserializationError jsonError = deserializeJson(configJson, buf.get());
        if (jsonError)
        { // Couldn't parse the saved config
          debugPrintln(String(F("SPIFFS: [ERROR] Failed to parse /config.json: ")) + String(jsonError.c_str()));
        }
        else
        {
          if (!configJson["haspNode"].isNull())
          {
            strcpy(haspNode, configJson["haspNode"]);
          }
          if (!configJson["groupName"].isNull())
          {
            strcpy(groupName, configJson["groupName"]);
          }
          if (!configJson["configUser"].isNull())
          {
            strcpy(configUser, configJson["configUser"]);
          }
          if (!configJson["configPassword"].isNull())
          {
            strcpy(configPassword, configJson["configPassword"]);
          }
          if (!configJson["motionPinConfig"].isNull())
          {
            strcpy(motionPinConfig, configJson["motionPinConfig"]);
          }
          if (!configJson["debugSerialEnabled"].isNull())
          {
            if (configJson["debugSerialEnabled"])
            {
              debugSerialEnabled = true;
            }
            else
            {
              debugSerialEnabled = false;
            }
          }
          if (!configJson["debugTelnetEnabled"].isNull())
          {
            if (configJson["debugTelnetEnabled"])
            {
              debugTelnetEnabled = true;
            }
            else
            {
              debugTelnetEnabled = false;
            }
          }
          if (!configJson["mdnsEnabled"].isNull())
          {
            if (configJson["mdnsEnabled"])
            {
              mdnsEnabled = true;
            }
            else
            {
              mdnsEnabled = false;
            }
          }
          String configJsonStr;
          serializeJson(configJson, configJsonStr);
          debugPrintln(String(F("SPIFFS: parsed json:")) + configJsonStr);
        }
      }
      else
      {
        debugPrintln(F("SPIFFS: [ERROR] Failed to read /config.json"));
      }
    }
    else
    {
      debugPrintln(F("SPIFFS: [WARNING] /config.json not found, will be created on first config save"));
    }
  }
  else
  {
    debugPrintln(F("SPIFFS: [ERROR] Failed to mount FS"));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片设置保存返回
*/
void configSaveCallback()
{ // Callback notifying us of the need to save config
  debugPrintln(F("SPIFFS: Configuration changed, flagging for save"));
  shouldSaveConfig = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片保存设置
*/
void configSave()
{ // Save the custom parameters to config.json
  nextionSetAttr("p[0].b[1].txt", "\"Saving\\rconfig\"");
  debugPrintln(F("SPIFFS: Saving config"));
  DynamicJsonDocument jsonConfigValues(1024);
  jsonConfigValues["haspNode"] = haspNode;
  jsonConfigValues["groupName"] = groupName;
  jsonConfigValues["configUser"] = configUser;
  jsonConfigValues["configPassword"] = configPassword;
  jsonConfigValues["motionPinConfig"] = motionPinConfig;
  jsonConfigValues["debugSerialEnabled"] = debugSerialEnabled;
  jsonConfigValues["debugTelnetEnabled"] = debugTelnetEnabled;
  jsonConfigValues["mdnsEnabled"] = mdnsEnabled;

  debugPrintln(String(F("SPIFFS: haspNode = ")) + String(haspNode));
  debugPrintln(String(F("SPIFFS: groupName = ")) + String(groupName));
  debugPrintln(String(F("SPIFFS: configUser = ")) + String(configUser));
  debugPrintln(String(F("SPIFFS: configPassword = ")) + String(configPassword));
  debugPrintln(String(F("SPIFFS: motionPinConfig = ")) + String(motionPinConfig));
  debugPrintln(String(F("SPIFFS: debugSerialEnabled = ")) + String(debugSerialEnabled));
  debugPrintln(String(F("SPIFFS: debugTelnetEnabled = ")) + String(debugTelnetEnabled));
  debugPrintln(String(F("SPIFFS: mdnsEnabled = ")) + String(mdnsEnabled));

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    debugPrintln(F("SPIFFS: Failed to open config file for writing"));
  }
  else
  {
    serializeJson(jsonConfigValues, configFile);
    configFile.close();
  }
  shouldSaveConfig = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* esp芯片清除保存
*/
void configClearSaved()
{ // Clear out all local storage
  nextionSendCmd("page 0");
  nextionSetAttr("p[0].b[1].txt", "\"Resetting\\rsystem...\"");
  debugPrintln(F("RESET: Formatting SPIFFS"));
  SPIFFS.format();
  debugPrintln(F("RESET: Clearing WiFiManager settings..."));
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  EEPROM.begin(512);
  debugPrintln(F("Clearing EEPROM..."));
  for (uint16_t i = 0; i < EEPROM.length(); i++)
  {
    EEPROM.write(i, 0);
  }
  nextionSetAttr("p[0].b[1].txt", "\"Rebooting\\rsystem...\"");
  debugPrintln(F("RESET: Rebooting device"));
  espReset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 404处理
*/
void webHandleNotFound()
{ // webServer 404
  debugPrintln(String(F("HTTP: Sending 404 to client connected from: ")) + webServer.client().remoteIP().toString());
  String httpMessage = "File Not Found\n\n";
  httpMessage += "URI: ";
  httpMessage += webServer.uri();
  httpMessage += "\nMethod: ";
  httpMessage += (webServer.method() == HTTP_GET) ? "GET" : "POST";
  httpMessage += "\nArguments: ";
  httpMessage += webServer.args();
  httpMessage += "\n";
  for (uint8_t i = 0; i < webServer.args(); i++)
  {
    httpMessage += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
  }
  webServer.send(404, "text/plain", httpMessage);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 首页处理
*/
void webHandleRoot()
{ // http://plate01/
  if (configPassword[0] != '\0')
  { //Request HTTP auth if configPassword is set
    if (!webServer.authenticate(configUser, configPassword))
    {
      return webServer.requestAuthentication();
    }
  }
  debugPrintln(String(F("HTTP: Sending root page to client connected from: ")) + webServer.client().remoteIP().toString());
  String httpMessage = FPSTR(HTTP_HEAD);
  httpMessage.replace("{v}", String(haspNode));
  httpMessage += FPSTR(HTTP_SCRIPT);
  httpMessage += FPSTR(HTTP_STYLE);
  httpMessage += String(HASP_STYLE);
//  httpMessage += FPSTR(HTTP_HEAD_END);
  httpMessage += String(F("<h1>"));
  httpMessage += String(haspNode);
  httpMessage += String(F("</h1>"));

  httpMessage += String(F("<form method='POST' action='saveConfig'>"));
  httpMessage += String(F("<b>WiFi SSID</b> <i><small>(required)</small></i><input id='wifiSSID' required name='wifiSSID' maxlength=32 placeholder='WiFi SSID' value='")) + String(WiFi.SSID()) + "'>";
  httpMessage += String(F("<br/><b>WiFi Password</b> <i><small>(required)</small></i><input id='wifiPass' required name='wifiPass' type='password' maxlength=64 placeholder='WiFi Password' value='")) + String("********") + "'>";
  httpMessage += String(F("<br/><br/><b>HASP Node Name</b> <i><small>(required. lowercase letters, numbers, and _ only)</small></i><input id='haspNode' required name='haspNode' maxlength=15 placeholder='HASP Node Name' pattern='[a-z0-9_]*' value='")) + String(haspNode) + "'>";
  httpMessage += String(F("<br/><br/><b>Group Name</b> <i><small>(required)</small></i><input id='groupName' required name='groupName' maxlength=15 placeholder='Group Name' value='")) + String(groupName) + "'>";

  httpMessage += String(F("'><br/><br/><b>HASP Admin Username</b> <i><small>(optional)</small></i><input id='configUser' name='configUser' maxlength=31 placeholder='Admin User' value='")) + String(configUser) + "'>";
  httpMessage += String(F("<br/><b>HASP Admin Password</b> <i><small>(optional)</small></i><input id='configPassword' name='configPassword' type='password' maxlength=31 placeholder='Admin User Password' value='"));
  if (strlen(configPassword) != 0)
  {
    httpMessage += String("********");
  }
  httpMessage += String(F("'><br/><hr><b>Motion Sensor Pin:&nbsp;</b><select id='motionPinConfig' name='motionPinConfig'>"));
  httpMessage += String(F("<option value='0'"));
  if (!motionPin)
  {
    httpMessage += String(F(" selected"));
  }
  httpMessage += String(F(">disabled/not installed</option><option value='D0'"));
  if (motionPin == D0)
  {
    httpMessage += String(F(" selected"));
  }
  httpMessage += String(F(">D0</option><option value='D1'"));
  if (motionPin == D1)
  {
    httpMessage += String(F(" selected"));
  }
  httpMessage += String(F(">D1</option><option value='D2'"));
  if (motionPin == D2)
  {
    httpMessage += String(F(" selected"));
  }
  httpMessage += String(F(">D2</option></select>"));

  httpMessage += String(F("<br/><b>Serial debug output enabled:</b><input id='debugSerialEnabled' name='debugSerialEnabled' type='checkbox'"));
  if (debugSerialEnabled)
  {
    httpMessage += String(F(" checked='checked'"));
  }
  httpMessage += String(F("><br/><b>Telnet debug output enabled:</b><input id='debugTelnetEnabled' name='debugTelnetEnabled' type='checkbox'"));
  if (debugTelnetEnabled)
  {
    httpMessage += String(F(" checked='checked'"));
  }
  httpMessage += String(F("><br/><b>mDNS enabled:</b><input id='mdnsEnabled' name='mdnsEnabled' type='checkbox'"));
  if (mdnsEnabled)
  {
    httpMessage += String(F(" checked='checked'"));
  }
  httpMessage += String(F("><br/><hr><button type='submit'>save settings</button></form>"));

  httpMessage += String(F("<hr><form method='get' action='firmware'>"));
  httpMessage += String(F("<button type='submit'>update firmware</button></form>"));

  httpMessage += String(F("<hr><form method='get' action='reboot'>"));
  httpMessage += String(F("<button type='submit'>reboot device</button></form>"));

  httpMessage += String(F("<hr><form method='get' action='resetConfig'>"));
  httpMessage += String(F("<button type='submit'>factory reset settings</button></form>"));

  httpMessage += String(F("<br/><b>LCD Active Page: </b>")) + String(nextionActivePage);
  httpMessage += String(F("<br/><b>CPU Frequency: </b>")) + String(ESP.getCpuFreqMHz()) + String(F("MHz"));
  httpMessage += String(F("<br/><b>Sketch Size: </b>")) + String(ESP.getSketchSize()) + String(F(" bytes"));
  httpMessage += String(F("<br/><b>Free Sketch Space: </b>")) + String(ESP.getFreeSketchSpace()) + String(F(" bytes"));
  httpMessage += String(F("<br/><b>Heap Free: </b>")) + String(ESP.getFreeHeap());
  httpMessage += String(F("<br/><b>Heap Fragmentation: </b>")) + String(ESP.getHeapFragmentation());
  httpMessage += String(F("<br/><b>ESP core version: </b>")) + String(ESP.getCoreVersion());
  httpMessage += String(F("<br/><b>IP Address: </b>")) + String(WiFi.localIP().toString());
  httpMessage += String(F("<br/><b>Signal Strength: </b>")) + String(WiFi.RSSI());
  httpMessage += String(F("<br/><b>Uptime: </b>")) + String(long(millis() / 1000));
  httpMessage += String(F("<br/><b>Last reset: </b>")) + String(ESP.getResetInfo());

  httpMessage += FPSTR(HTTP_END);
  webServer.send(200, "text/html", httpMessage);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 保存设置处理
*/
void webHandleSaveConfig()
{ // http://plate01/saveConfig
  if (configPassword[0] != '\0')
  { //Request HTTP auth if configPassword is set
    if (!webServer.authenticate(configUser, configPassword))
    {
      return webServer.requestAuthentication();
    }
  }
  debugPrintln(String(F("HTTP: Sending /saveConfig page to client connected from: ")) + webServer.client().remoteIP().toString());
  String httpMessage = FPSTR(HTTP_HEAD);
  httpMessage.replace("{v}", String(haspNode));
  httpMessage += FPSTR(HTTP_SCRIPT);
  httpMessage += FPSTR(HTTP_STYLE);
  httpMessage += String(HASP_STYLE);

  bool shouldSaveWifi = false;
  // Check required values
  if (webServer.arg("wifiSSID") != "" && webServer.arg("wifiSSID") != String(WiFi.SSID()))
  { // Handle WiFi update
    shouldSaveConfig = true;
    shouldSaveWifi = true;
    webServer.arg("wifiSSID").toCharArray(wifiSSID, 32);
    if (webServer.arg("wifiPass") != String("********"))
    {
      webServer.arg("wifiPass").toCharArray(wifiPass, 64);
    }
  }
  if (webServer.arg("haspNode") != "" && webServer.arg("haspNode") != String(haspNode))
  { // Handle haspNode
    shouldSaveConfig = true;
    String lowerHaspNode = webServer.arg("haspNode");
    lowerHaspNode.toLowerCase();
    lowerHaspNode.toCharArray(haspNode, 16);
  }
  if (webServer.arg("groupName") != "" && webServer.arg("groupName") != String(groupName))
  { // Handle groupName
    shouldSaveConfig = true;
    webServer.arg("groupName").toCharArray(groupName, 16);
  }
  if (webServer.arg("configUser") != String(configUser))
  { // Handle configUser
    shouldSaveConfig = true;
    webServer.arg("configUser").toCharArray(configUser, 32);
  }
  if (webServer.arg("configPassword") != String("********"))
  { // Handle configPassword
    shouldSaveConfig = true;
    webServer.arg("configPassword").toCharArray(configPassword, 32);
  }
  if (webServer.arg("motionPinConfig") != String(motionPinConfig))
  { // Handle motionPinConfig
    shouldSaveConfig = true;
    webServer.arg("motionPinConfig").toCharArray(motionPinConfig, 3);
  }
  if ((webServer.arg("debugSerialEnabled") == String("on")) && !debugSerialEnabled)
  { // debugSerialEnabled was disabled but should now be enabled
    shouldSaveConfig = true;
    debugSerialEnabled = true;
  }
  else if ((webServer.arg("debugSerialEnabled") == String("")) && debugSerialEnabled)
  { // debugSerialEnabled was enabled but should now be disabled
    shouldSaveConfig = true;
    debugSerialEnabled = false;
  }
  if ((webServer.arg("debugTelnetEnabled") == String("on")) && !debugTelnetEnabled)
  { // debugTelnetEnabled was disabled but should now be enabled
    shouldSaveConfig = true;
    debugTelnetEnabled = true;
  }
  else if ((webServer.arg("debugTelnetEnabled") == String("")) && debugTelnetEnabled)
  { // debugTelnetEnabled was enabled but should now be disabled
    shouldSaveConfig = true;
    debugTelnetEnabled = false;
  }
  if ((webServer.arg("mdnsEnabled") == String("on")) && !mdnsEnabled)
  { // mdnsEnabled was disabled but should now be enabled
    shouldSaveConfig = true;
    mdnsEnabled = true;
  }
  else if ((webServer.arg("mdnsEnabled") == String("")) && mdnsEnabled)
  { // mdnsEnabled was enabled but should now be disabled
    shouldSaveConfig = true;
    mdnsEnabled = false;
  }

  if (shouldSaveConfig)
  { // Config updated, notify user and trigger write to SPIFFS
    httpMessage += String(F("<meta http-equiv='refresh' content='15;url=/' />"));
//    httpMessage += FPSTR(HTTP_HEAD_END);
    httpMessage += String(F("<h1>")) + String(haspNode) + String(F("</h1>"));
    httpMessage += String(F("<br/>Saving updated configuration values and restarting device"));
    httpMessage += FPSTR(HTTP_END);
    webServer.send(200, "text/html", httpMessage);

    configSave();
    if (shouldSaveWifi)
    {
      debugPrintln(String(F("CONFIG: Attempting connection to SSID: ")) + webServer.arg("wifiSSID"));
      espWifiSetup();
    }
    espReset();
  }
  else
  { // No change found, notify user and link back to config page
    httpMessage += String(F("<meta http-equiv='refresh' content='3;url=/' />"));
//    httpMessage += FPSTR(HTTP_HEAD_END);
    httpMessage += String(F("<h1>")) + String(haspNode) + String(F("</h1>"));
    httpMessage += String(F("<br/>No changes found, returning to <a href='/'>home page</a>"));
    httpMessage += FPSTR(HTTP_END);
    webServer.send(200, "text/html", httpMessage);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 重置处理
*/
void webHandleResetConfig()
{ // http://plate01/resetConfig
  if (configPassword[0] != '\0')
  { //Request HTTP auth if configPassword is set
    if (!webServer.authenticate(configUser, configPassword))
    {
      return webServer.requestAuthentication();
    }
  }
  debugPrintln(String(F("HTTP: Sending /resetConfig page to client connected from: ")) + webServer.client().remoteIP().toString());
  String httpMessage = FPSTR(HTTP_HEAD);
  httpMessage.replace("{v}", String(haspNode));
  httpMessage += FPSTR(HTTP_SCRIPT);
  httpMessage += FPSTR(HTTP_STYLE);
  httpMessage += String(HASP_STYLE);
//  httpMessage += FPSTR(HTTP_HEAD_END);

  if (webServer.arg("confirm") == "yes")
  { // User has confirmed, so reset everything
    httpMessage += String(F("<h1>"));
    httpMessage += String(haspNode);
    httpMessage += String(F("</h1><b>Resetting all saved settings and restarting device into WiFi AP mode</b>"));
    httpMessage += FPSTR(HTTP_END);
    webServer.send(200, "text/html", httpMessage);
    delay(1000);
    configClearSaved();
  }
  else
  {
    httpMessage += String(F("<h1>Warning</h1><b>This process will reset all settings to the default values and restart the device.  You may need to connect to the WiFi AP displayed on the panel to re-configure the device before accessing it again."));
    httpMessage += String(F("<br/><hr><br/><form method='get' action='resetConfig'>"));
    httpMessage += String(F("<br/><br/><button type='submit' name='confirm' value='yes'>reset all settings</button></form>"));
    httpMessage += String(F("<br/><hr><br/><form method='get' action='/'>"));
    httpMessage += String(F("<button type='submit'>return home</button></form>"));
    httpMessage += FPSTR(HTTP_END);
    webServer.send(200, "text/html", httpMessage);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 重启处理
*/
void webHandleReboot()
{ // http://plate01/reboot
  if (configPassword[0] != '\0')
  { //Request HTTP auth if configPassword is set
    if (!webServer.authenticate(configUser, configPassword))
    {
      return webServer.requestAuthentication();
    }
  }
  debugPrintln(String(F("HTTP: Sending /reboot page to client connected from: ")) + webServer.client().remoteIP().toString());
  String httpMessage = FPSTR(HTTP_HEAD);
  httpMessage.replace("{v}", (String(haspNode) + " HASP reboot"));
  httpMessage += FPSTR(HTTP_SCRIPT);
  httpMessage += FPSTR(HTTP_STYLE);
  httpMessage += String(HASP_STYLE);
  httpMessage += String(F("<meta http-equiv='refresh' content='10;url=/' />"));
//  httpMessage += FPSTR(HTTP_HEAD_END);
  httpMessage += String(F("<h1>")) + String(haspNode) + String(F("</h1>"));
  httpMessage += String(F("<br/>Rebooting device"));
  httpMessage += FPSTR(HTTP_END);
  webServer.send(200, "text/html", httpMessage);
  debugPrintln(F("RESET: Rebooting device"));
  nextionSendCmd("page 0");
  nextionSetAttr("p[0].b[1].txt", "\"Rebooting...\"");
  espReset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 调试输出换行
*/
void debugPrintln(String debugText)
{ // Debug output line of text to our debug targets
  String debugTimeText = "[+" + String(float(millis()) / 1000, 3) + "s] " + debugText;
  Serial.println(debugTimeText);
  if (debugSerialEnabled)
  {
    SoftwareSerial debugSerial(SW_SERIAL_UNUSED_PIN, 1); // 17==nc for RX, 1==TX pin
    debugSerial.begin(115200);
    debugSerial.println(debugTimeText);
    debugSerial.flush();
  }
  if (debugTelnetEnabled)
  {
    if (telnetClient.connected())
    {
      telnetClient.println(debugTimeText);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 调试输出
*/
void debugPrint(String debugText)
{ // Debug output single character to our debug targets (DON'T USE THIS!)
  // Try to avoid using this function if at all possible.  When connected to telnet, printing each
  // character requires a full TCP round-trip + acknowledgement back and execution halts while this
  // happens.  Far better to put everything into a line and send it all out in one packet using
  // debugPrintln.
  if (debugSerialEnabled)
    Serial.print(debugText);
  {
    SoftwareSerial debugSerial(SW_SERIAL_UNUSED_PIN, 1); // 17==nc for RX, 1==TX pin
    debugSerial.begin(115200);
    debugSerial.print(debugText);
    debugSerial.flush();
  }
  if (debugTelnetEnabled)
  {
    if (telnetClient.connected())
    {
      telnetClient.print(debugText);
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// UTF8-Decoder: convert UTF8-string to extended ASCII http://playground.arduino.cc/main/Utf8ascii
// Workaround for issue here: https://github.com/home-assistant/home-assistant/issues/9528
// Nextion claims that "Unicode and UTF will not be among the supported encodings", so this should
// be safe to run against all attribute values coming in.
static byte c1; // Last character buffer
byte utf8ascii(byte ascii)
{ // Convert a single Character from UTF8 to Extended ASCII. Return "0" if a byte has to be ignored.
  if (ascii < 128)
  { // Standard ASCII-set 0..0x7F handling
    c1 = 0;
    return (ascii);
  }
  // get previous input
  byte last = c1; // get last char
  c1 = ascii;     // remember actual character
  switch (last)
  { // conversion depending on first UTF8-character
  case 0xC2:
    return (ascii);
    break;
  case 0xC3:
    return (ascii | 0xC0);
    break;
  case 0x82:
    if (ascii == 0xAC)
      return (0x80); // special case Euro-symbol
  }
  return (0); // otherwise: return zero, if character has to be ignored
}

String utf8ascii(String s)
{ // convert String object from UTF8 String to Extended ASCII
  String r = "";
  char c;
  for (uint16_t i = 0; i < s.length(); i++)
  {
    c = utf8ascii(s.charAt(i));
    if (c != 0)
      r += c;
  }
  return r;
}

void utf8ascii(char *s)
{ // In Place conversion UTF8-string to Extended ASCII (ASCII is shorter!)
  uint16_t k = 0;
  char c;
  for (uint16_t i = 0; i < strlen(s); i++)
  {
    c = utf8ascii(s[i]);
    if (c != 0)
      s[k++] = c;
  }
  s[k] = 0;
}
