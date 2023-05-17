#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <WiFiManager.h>

////////////////////////////////DISPLAY//////////////////////////////////
#define I2C_SDA 15
#define I2C_SCL 14
TwoWire I2Cbus = TwoWire(0);

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

// Initialize Telegram BOT
String BOTtoken = "6292139229:AAFcg6f7pZKVx1CXlsYiRw4KZ1M7lAMiprI";  // your Bot Token (Get from Botfather)
// message you
String CHAT_ID = "6263467376";

bool sendPhoto = false;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

//#define FLASH_LED_PIN 4

int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


//////////////////////////////////MQ3 SENSOR//////////////////////////////
#define Drunk 1900  // Define min value that we consider drunk
#define MQ3pin 13

//WiFiClient client;
//HTTPClient httpClient;

WiFiServer server(80);  

int sensorValue;  //variable to store sensor value

//int Engine = 0;

int BUTTON_PIN = 12; //toggle button
int lastButtonState;    // the previous state of button
int currentButtonState; // the current state of button
int buttonClick;
int enginStart = 0;
int button1;
int buzzer = 2;
int k;

/////////////////////////////////////Camera Configuration//////////////////////////
void configInitCamera(){
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
}

//////////////////////////////////////////////////////handel msg//////////////////////////////////////
void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = "6263467376";
    if (chat_id != CHAT_ID){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = "/photo";
    //
      Serial.println(text);
    
    String from_name = "rajath";
    /*if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }*/
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
      photosend();
    }
  }
}

//////////////////////////////////////////////////////////sEND pHOTO/////////////////////////
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(50);
    ESP.restart();
    return "Camera capture failed";
  } 
  
  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--Electro\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--Electro\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Electro--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=Electro");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

///////////////////////////////////////////////wifi//////////////////////////////////////////
void wifisetup(){
       WiFiManager wifiManager;
  //wifiManager.resetSettings();
  wifiManager.autoConnect("Alcohol Sensor");
  Serial.println("Connected.");
  server.begin();
  //Serial.println(WiFi.localIP());
      clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
      while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(200);
      }
      Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
}
///////////////////////////////////////////////////photo//////////////////////////////////////
void photosend(){
  
      if (sendPhoto) {
        buzzers();
          
            Serial.println("Preparing photo");
            sendPhotoTelegram(); 
            sendPhoto = false;             
            }
            Serial.print("lastTime & BotrequestDelay =");
            Serial.println(lastTimeBotRan+""+botRequestDelay);
            if (millis() > lastTimeBotRan + botRequestDelay)  {
              int numNewMessages = 1;
              int i = 0;
              i += 1;
              while (numNewMessages) {
                Serial.println("got response");
                handleNewMessages(numNewMessages);
                numNewMessages = bot.getUpdates(bot.last_message_received + 1);
              }
              lastTimeBotRan = millis();
              
            }
      
            
}

void buzzers(){

  unsigned char i, j ;// define variables
  int k = 1;
while (k <= 5)
{
 k = k+1; 
for (i = 0; i <80; i++)
{
digitalWrite (buzzer, LOW); // Turn buzzer ON
delay (1) ;// Delay 1ms
digitalWrite (buzzer, HIGH);// turn buzzer OFF
delay (1) ;// delay ms
}
for (i = 0; i <100; i++) // new frequency
{
digitalWrite (buzzer, LOW);// turn buzzer ON
delay (2) ;// delay 2ms
digitalWrite (buzzer, HIGH);// turn buzzer OFF
delay (2) ;// delay 2ms
}
}
}
///////////////////////////////////////////// setup()///////////////////////////////////////////
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    // Init Serial Monitor

  Serial.begin(115200);

  // Config and init the camera
  configInitCamera();

  ///////////////////////////////////////////////////////////////////////////////////////////////
  buttonClick = 0;
  pinMode(BUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
  pinMode(13, OUTPUT); //motor pin
  pinMode (buzzer, OUTPUT);


  //pinMode(MQ3pin, INPUT);

  digitalWrite(16, LOW);
  digitalWrite(BUTTON_PIN, LOW);
  currentButtonState = digitalRead(BUTTON_PIN);
	
  //Serial.begin(115200); // sets the serial port to 9600

	

/////////////////////////////////////////////////////////oled setup//////////////////////////////////////
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);

  //Serial.println("Initialize display");
 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.printf("SSD1306 OLED display failed to initalize");
    while (true);
  }

  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("Alcohol Detector");
  display.display();
  //display.clearDisplay();
  display.setCursor(0, 25);
  display.setTextSize(1);
  display.print("please wait...!");
  display.display();
  Serial.println("MQ3 warming up!");
	delay(15000); // allow the MQ3 to warm up
  //////////////////////////////////////////
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Ready...");
  display.display();
  display.setCursor(0,24);
  display.setTextSize(1);
  display.print("Start Engine");
  display.display();
  display.clearDisplay();

  
}
  

/////////////////////////////////////////////////LOOP///////////////////////////////////////////////////



void loop() {

  sensorValue = analogRead(MQ3pin); // read analog input pin 0

	Serial.print("Sensor Value: ");
	Serial.println(sensorValue);

	
	delay(100);

  //return;

  button1 = digitalRead(BUTTON_PIN);
  Serial.println(button1);
  


	if(buttonClick == 1 || button1== 0 )
  {
    display.clearDisplay();
    buttonClick = 1;

	  if (sensorValue < Drunk) {
      enginStart = 1;
	  	Serial.println("  |  Status: NOT DRUNK");
      display.setCursor(0, 10);
      display.setTextSize(1);
      digitalWrite(16, HIGH);
      //display.startscrollleft(0x00, 0x0F);
      display.print("NOT DRUNK = ");
      display.println(sensorValue);
      display.display();
      display.clearDisplay();
      delay(30); 
      }     

	    else if (sensorValue > Drunk && enginStart == 1) {
       display.clearDisplay();
           display.setCursor(0, 10);
           display.setTextSize(1);
            display.println("CLICKING PHOTO");
            display.display();
           // display.startscrollleft(0x00, 0x0F);
            delay(3000);
            display.clearDisplay();
            display.stopscroll();
      //////////////////////////wifi///////////////////
        buzzers();
        wifisetup();
        photosend(); 
        }
      
      else if (sensorValue > Drunk && enginStart == 0)
      {
        display.clearDisplay();
		      Serial.println("  |  Status: DRUNK CANNOT START");
          digitalWrite(16, LOW);
          display.setCursor(0, 10);
          display.setTextSize(1);
          display.println("CLICKING PHOTO");
          display.display();
         // display.startscrollleft(0x00, 0x0F);
          delay(3000);
         display.clearDisplay();
          /////for loop//////
          delay(100);
          //display.clearDisplay();
        //////////////////////////wifi///////////////////
        buzzers();
        wifisetup();
        photosend();
    
      }
  }    
delay(30); // wait 2s for next reading
} 
	