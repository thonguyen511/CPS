#include "BH1750.h"
#include "Wire.h"
#include <Arduino.h>
#include <WebSocketsServer.h> //import for websocket
#include <ArduinoJson.h> //data Json

const char *ssid =  "esp";   //Wifi SSID (Name)
const char *pass =  "123456789"; //wifi password

WebSocketsServer webSocket = WebSocketsServer(81,"*"); //websocket init with port 81 // allow cors



BH1750 bh1750_a;
BH1750 bh1750_b;

// PID controller constants
float kp = 0.1;   // Proportional gain
float ki = 0.04;   // Integral gain
float kd = 0.02;   // Derivative gain
const int led = 26;
const int ledChannel=0;
int motionDetected;
int brightness ;
float light_level_a = 0;
float light_level_b = 0;

// PID variables
float setpoint = 30; // Desired stable difference
float output;
float error, lastError;
float integral, derivative;

// Time variables
unsigned long previousMillis = 0;
const unsigned long interval = 500; 

StaticJsonDocument<500> TempDoc;

void setup() {
  Serial.begin(115200);

  Serial.println("Starting wifi...");
  IPAddress apIP(192, 168, 99, 100);   //Static IP for wifi gateway
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)); //set Static IP gateway on NodeMCU
  WiFi.softAP(ssid, pass); //turn on WIFI
  
  webSocket.begin();

  //websocket Begin
  webSocket.onEvent(webSocketEvent); //set event listener
  Serial.println("Websocket is started");

  Wire.begin(18, 19);
  Wire1.begin(21, 22);
  bh1750_a.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  bh1750_b.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire1);

  // Initialize PID variables

  output = 0;
  error = 0;
  lastError = 0;
  integral = 0;
  derivative = 0;

  pinMode(12, INPUT); // Set HC-SR501 PIR motion sensor pin as input
  pinMode(led, OUTPUT);
  ledcSetup(ledChannel, 5000, 8);
  ledcAttachPin(led, ledChannel);
  ledcWrite(ledChannel, 255);
}



void loop() {

  webSocket.loop();
  unsigned long currentMillis = millis();

  // Read data from HC-SR501 PIR motion sensor
  motionDetected = digitalRead(12);
  if(motionDetected){
  // Perform PID update at specified interval

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
  
  if (bh1750_a.measurementReady()) {
    light_level_a = bh1750_a.readLightLevel();
    // độ sáng cảm biến a đặt gần đèn
  }

  if (bh1750_b.measurementReady()) {
    light_level_b = bh1750_b.readLightLevel();
    // độ sáng cảm biến b đặt gần mặt đường
  }
  float diff=(light_level_a-light_level_b)*(50/light_level_b)*(1/256);
  // chênh lệch giá trị đo được giữa 2 cảm biến nhân với một tỉ lệ thay đổi theo giá trị cảm biến b 
  // tăng diff mạnh hơn nếu giá trị cảm biến b nhỏ hơn 50 (50 là ngưỡng sáng tối thiểu cần để nhìn tốt) -> âm error -> giảm brightness
  // giảm diff vào ban ngày ( cảm biến b có giá trị lớn hơn 50 ) -> dương error -> tăng brightness
  // *(1/256) để giảm giá trị dữ liệu thô xuống con số phù hợp với giá trị 4 bit (dữ liệu thô từ cảm biến là 16 bit)
  // Vào buổi tối : 0.001 - 0.02 Lux
  // Ánh trăng : 0.02 - 0.3 lux
  // Trời nhiều mây trong nhà : 5 - 50 lux
  // Trời nhiều mây ngoài trời : 50 - 500 lux
  // Trời nắng trong nhà : 100 - 1000 lux
  // Ánh sáng cần thiết để đọc sách: 50 - 60 lux
  

  // PID calculations
  error = setpoint - diff;
  integral = integral+ error*interval; 
  derivative = (error - lastError)*(1/interval);
  lastError = error;

  output = (kp * error) + (ki * integral) + (kd * derivative);

  output = constrain(output,0,255); // set min max đầu ra 
  if(output == 0 || output == 255){
    integral = 0; // đặt lại integral về 0 nếu output đã chạm min/max để tránh integral -> vô hạn khi vẫn xảy ra lỗi  
  }
  brightness=output;

  TempDoc["kp"]=kp; // p
  TempDoc["ki"]=ki; // i
  TempDoc["kd"]=kd; // d
  TempDoc["setpoint"]=setpoint; //s
  TempDoc["output"]=output; 
  TempDoc["brightness"]=brightness; 
  TempDoc["motionState"]=motionDetected;
  TempDoc["lxA"]=light_level_a;
  TempDoc["lxB"]=light_level_b;
  char msg[256];
  serializeJson(TempDoc, msg);//dich ra json
  webSocket.broadcastTXT(msg);//gui websocket
  
  }
  
    ledcWrite(ledChannel,brightness); 
  }else{  
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      TempDoc["kp"]=kp; // p   
      TempDoc["ki"]=ki; // i
      TempDoc["kd"]=kd; // d
      TempDoc["setpoint"]=setpoint; //s
      TempDoc["output"]=output; 
      TempDoc["brightness"]=brightness; 
      TempDoc["motionState"]=motionDetected;
      TempDoc["lxA"]=light_level_a;
      TempDoc["lxB"]=light_level_b;
      char msg[256];
      serializeJson(TempDoc, msg);//dich ra json
      webSocket.broadcastTXT(msg);//gui websocket
    }
      ledcWrite(ledChannel,0);//off  
  } 
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  //webscket event method
  String cmd = "";
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("disconnected");

      break;
    case WStype_CONNECTED: {
        Serial.println("a client connected");
        Serial.println(webSocket.remoteIP(num).toString());

        webSocket.sendTXT(num, "connected");
      }
      break;
    case WStype_TEXT:
      {cmd = "";
      for (int i = 0; i < length; i++) {
        cmd = cmd + (char) payload[i];
      } //merging payload to single string
      Serial.print("Text received: ");
      Serial.println(cmd);

      // Parse the JSON data
      StaticJsonDocument<200> jsonDoc;
      DeserializationError error = deserializeJson(jsonDoc, cmd);

      // Check if parsing succeeded
      if (error) {
        // Serial.print("JSON parsing failed: ");
        // Serial.println(error.c_str());
        break;
      }
      // Retrieve PID controller parameters and setpoint
      if (jsonDoc.containsKey("kp")) {
        kp = jsonDoc["kp"].as<float>();
        // Serial.print(kp);
      }
      if (jsonDoc.containsKey("ki")) {
        ki = jsonDoc["ki"].as<float>();
        // Serial.print(ki);
      }
      if (jsonDoc.containsKey("kd")) {
        kd = jsonDoc["kd"].as<float>();
        // Serial.print(kd);
      }
      if (jsonDoc.containsKey("setpoint")) {
        setpoint = jsonDoc["setpoint"].as<float>();
        // Serial.print(setpoint);
      }}
      break;



    case WStype_FRAGMENT_TEXT_START:
      break;
    case WStype_FRAGMENT_BIN_START:
      break;
    case WStype_BIN:
      break;
    default:
      break;
  }
}