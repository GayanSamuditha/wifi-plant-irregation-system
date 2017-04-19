#include <ConnectThings_ESP8266.h>
#include <dummy.h>
#include <Esp8266EasyIoT.h>
#include <SoftwareSerial.h> 

#include <DHT.h>  

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define HUMIDITY_SENSOR_DIGITAL_PIN 2

//Esp8266EasyIoT esp; 

#define MS_IN_SEC  1000 // 1S  
SoftwareSerial serialEsp(10, 11);


#include "UbidotsMicroESP8266.h"
#define TOKEN  "sjgs45k704meorim80"  // Put here your Ubidots TOKEN
#define WIFISSID "Hetti-WiFi"
#define PASSWORD "samsung123"

#define LEAK_PIN  2  // Arduino Digital I/O pin number
#define CHILD_ID_LEAK 0

Esp8266EasyIoTMsg msgLeak(CHILD_ID_LEAK, V_DIGITAL_VALUE);
//Esp8266EasyIoTMsg msgHum(CHILD_ID_LEAK, V_LEAK); // supported in esp >= V1.1 lib

Ubidots client(TOKEN);

void setup(){
    Serial.begin(115200);
    delay(10);
    client.wifiConnection("Hetti-WiFi",samsung123 );
}
void loop(){
    float value = analogRead(A0);
    client.add("Temperature", value);
    client.sendAll(true);
}
DHT dht;
float lastTemp;
float lastHum;

Esp8266EasyIoTMsg msgHum(CHILD_ID_HUM, V_HUM);
Esp8266EasyIoTMsg msgTemp(CHILD_ID_TEMP, V_TEMP);


void setup()
{
  serialEsp.begin(9600);
  Serial.begin(115200);  

  Serial.println("EasyIoTEsp init");


  esp.begin(NULL, 3, &serialEsp, &Serial);
  //esp.begin(NULL, &serialEsp);
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 

  pinMode(13, OUTPUT);

//  Serial.println("present S_HUM");
  esp.present(CHILD_ID_HUM, S_HUM);

//  Serial.println("present S_TEMP");
  esp.present(CHILD_ID_TEMP, S_TEMP);

}

void loop()
{  
  while(!esp.process());

  delay(dht.getMinimumSamplingPeriod());

  while(!esp.process());

  float temperature = dht.getTemperature();
  if (isnan(temperature)) {
    Serial.println("Failed reading temperature from DHT");
  } 
  else if (temperature != lastTemp) 
  {
    lastTemp = temperature;
    esp.send(msgTemp.set(temperature, 1));
    Serial.print("T: ");
    Serial.println(temperature);
  }

  float humidity = dht.getHumidity();
  if (isnan(humidity)) {
    Serial.println("Failed reading humidity from DHT");
  } 
  else if (humidity != lastHum) 
  {
    lastHum = humidity;
    esp.send(msgHum.set(humidity, 1));
    Serial.print("H: ");
    Serial.println(humidity);
  }
}




int lastLeakValue = -1;
// irrigator state
typedef enum {
  s_idle             = 0,  // irrigation idle
  s_irrigation_start = 1,  // start irrigation
  s_irrigation       = 2,  // irrigate
  s_irrigation_stop  = 3,  // irrigation stop
} e_state;


#define CHILD_ID_SWITCH_IRRIGATE        0
#define CHILD_ID_AUTO_MODE              1
#define CHILD_ID_SOIL_HUMIDITY          2
#define CHILD_ID_SOIL_HUMIDITY_AO       3

#define HUM_ANALOG_PIN        A0
#define MOTOR_PUMP_PIN        3
#define ESP_RESET_PIN         2
#define ESP_ENABLE_PIN        5

#define MAX_ANALOG_VAL         1023
#define MIN_ANALOG_VAL         430
#define IRRIGATION_TIME        15 // irrigation time in sec
#define IRRIGATION_PAUSE_TIME  300 // irrigation pause time in sec - only for auto irrigator


int lastAnalogReading;
unsigned long startTime;
bool pumpMotor;
int soilHum;

int state;
bool autoMode;
int irrigatorCounter;
int soilHumidityTreshold;

Esp8266EasyIoTMsg msgHum(CHILD_ID_SOIL_HUMIDITY, V_HUM);
Esp8266EasyIoTMsg msgMotorPump(CHILD_ID_SWITCH_IRRIGATE, V_DIGITAL_VALUE);

void setup()
{
  state = s_idle;
  pumpMotor = false;
  startTime = millis();
  soilHum = -1; 
  autoMode = 0;
  soilHumidityTreshold = 0;

  // ESP enable for custom board
  pinMode(ESP_ENABLE_PIN, OUTPUT);
  digitalWrite(ESP_ENABLE_PIN, HIGH);

  
  serialEsp.begin(9600);
  Serial.begin(115200);  

  Serial.println("EasyIoTEsp init");
  esp.begin(incomingMessage, ESP_RESET_PIN, &serialEsp, &Serial);
  //esp.begin(incomingMessage, 3, &serialEsp);

  
  pinMode(MOTOR_PUMP_PIN, OUTPUT);
  digitalWrite(MOTOR_PUMP_PIN, LOW);

  esp.present(CHILD_ID_SWITCH_IRRIGATE, S_DIGITAL_OUTPUT); // irrigation mottor switch
  esp.present(CHILD_ID_AUTO_MODE, S_DIGITAL_OUTPUT);     // irrigator mode - 1 auto, 0 - manual
  esp.present(CHILD_ID_SOIL_HUMIDITY, S_HUM);       // soil humidity
  esp.present(CHILD_ID_SOIL_HUMIDITY_AO, S_HUM_AO);        // soil humidity treshold
  
  
  // request treshold humidity
  esp.request(CHILD_ID_SOIL_HUMIDITY_AO, V_HUM);
  // request auto mode
  esp.request(CHILD_ID_AUTO_MODE, V_DIGITAL_VALUE);

  // read AI value
  lastAnalogReading = analogRead(HUM_ANALOG_PIN);  
}

void loop()
{
   while(!esp.process());
  
  if (IsTimeout())
  {
    startTime = millis();
    // process every second
    int aireading = analogRead(HUM_ANALOG_PIN);

    Serial.println("Analog value: ");
    Serial.println(aireading);
    // filter 
    lastAnalogReading += (aireading - lastAnalogReading) / 10;  
    Serial.println(lastAnalogReading); 
   
   // calculate soil humidity in % 
   int newSoilHum = map(lastAnalogReading, MIN_ANALOG_VAL, 1023, 100, 0);  
   Serial.println(newSoilHum); 
   
       
   // limit to 0-100%
   newSoilHum = max(0, newSoilHum);
   newSoilHum = min(100, newSoilHum);
   
   // report soil humidity if changed
   if (soilHum != newSoilHum)
   {
     soilHum = newSoilHum;
     esp.send(msgHum.set(soilHum)); 
   }
   
   
   // irrigator state machine
   switch(state)
   {
     case s_idle:     
       if (irrigatorCounter <= IRRIGATION_PAUSE_TIME)
         irrigatorCounter++;
       
       if (irrigatorCounter >= IRRIGATION_PAUSE_TIME && autoMode)
       {

SoftwareSerial serialEsp(10, 11);
         if (soilHum <= soilHumidityTreshold)
           state = s_irrigation_start;       
       }         
       break;
     case s_irrigation_start:
       irrigatorCounter = 0;
       digitalWrite(MOTOR_PUMP_PIN, HIGH);
       esp.send(msgMotorPump.set((uint8_t)1));
       state = s_irrigation;
       break;
     case s_irrigation:
       if (irrigatorCounter++ > IRRIGATION_TIME)
         state = s_irrigation_stop;
       break;
     case s_irrigation_stop:
       irrigatorCounter = 0;
       state = s_idle;
       esp.send(msgMotorPump.set((uint8_t)0));
       digitalWrite(MOTOR_PUMP_PIN, LOW);
       break;
   }
  }
}

void incomingMessage(const Esp8266EasyIoTMsg &message) {
  // We only expect one type of message from controller. But we better check anyway.

  Serial.println("");
  Serial.print("Incoming change for sensor:");
  Serial.println(message.sensor);

  if (message.type == V_DIGITAL_VALUE) {    
    // process mode commnd
    if (message.sensor == CHILD_ID_AUTO_MODE)
      autoMode = message.getBool();
    // process motor switch command  
    else if (message.sensor == CHILD_ID_SWITCH_IRRIGATE)        
    {
      if (message.getBool()==true)
        state = s_irrigation_start;
      else
        state = s_irrigation_stop;
    }
  }else if (message.type == V_HUM && message.sensor == CHILD_ID_SOIL_HUMIDITY_AO) {    
    // set humidity treshold for automatic irrigation
    soilHumidityTreshold = message.getInt();
  }  
}

boolean IsTimeout()
{
  unsigned long now = millis();
  if (startTime <= now)
  {
    if ( (unsigned long)(now - startTime )  < MS_IN_SEC ) 
      return false;
  }
  else
  {
    if ( (unsigned long)(startTime - now) < MS_IN_SEC ) 
      return false;
  }

  return true;
}

void setup()
{
  serialEsp.begin(9600);
  Serial.begin(115200);  

  Serial.println("EasyIoTEsp init");
  esp.begin(NULL, 3, &serialEsp, &Serial);

  pinMode(LEAK_PIN, INPUT);

  esp.present(CHILD_ID_LEAK, S_LEAK);
}

void loop()
{
  esp.process();
  
  // Read digital pin value
  int leakValue = digitalRead(LEAK_PIN); 
  // send if changed
  if (leakValue != lastLeakValue) {    
    Serial.println(leakValue);
    esp.send(msgLeak.set(leakValue==0?0:1));
    lastLeakValue = leakValue;
  }  
}


