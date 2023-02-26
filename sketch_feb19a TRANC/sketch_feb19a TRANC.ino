//..........................RESSIVER  code:..............................//
#include <FirebaseESP32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
//..........................General Settings..............................//
// Deffinig the projec's credentials wifi, Firebase, variables for calck & messering:_  
#define FIREBASE_HOST "https://fin-pro---gyro-testing-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyByeHkHsDOy0cK6ny2wVKFa5melfzUkPMA"
#define WIFI_SSID "Khapchikwifiv4"  // Deffinig the WIFI SSID (WiFi name) 
#define WIFI_PASSWORD "xanadu22432" // Deffinig the WIFI PASSWORD 

//Provide the token generation process info:_
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

uint8_t broadcastAddress[] = {0x48,0x3f,0xda,0x87,0xf0,0x6b}; // transmitter MAC address.
// uint8_t broadcastAddress[] = {0xff,0xff,0xff,0xff,0xff,0xff}; // Brodcast to all boards. 

FirebaseData fbdo;       // deffining obj for fire_base .& for the esp_now
FirebaseJson json;       // deffining json for FirebaseJson.
FirebaseAuth auth;       // Provide the token generation process info.
FirebaseConfig config;   // Provide the RTDB payload with helper functions.
Adafruit_MPU6050 mpu;    // I2C

// deffinig variables for this prijec:
// olso deffinig Struct for the data that will be received over ESPNow:_
float c   = 0;   // Calculated side
float x0  = 0;   // Measured pich angle 
float x1  = 0;   // Received pich angle 
float yo0 = 0;   // Measured yaw angle 
float z0  = 0;   // Measured roll angle
float w   = 0;   // Differential angle -> [360 - (x0 + xn)]
//......................................................................//

void  calck (){return;}


//......................................................................//
 
void setup() {
// Initialize serial communication and wait for the serial console to open
Serial.begin(115200);
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.print("Connecting to Wi-Fi");
// Wait for WiFi connection
while (WiFi.status() != WL_CONNECTED){
  Serial.print(".");
  delay(300);
}

Serial.print("Connected with IP: ");
Serial.println(WiFi.localIP());

config.api_key = FIREBASE_AUTH;
config.database_url = FIREBASE_HOST;

if (Firebase.signUp(&config, &auth, "", "")){
  Serial.println("ok");} 
else{Serial.printf("%s\n", config.signer.signupError.message.c_str());}

/* Assign the callback function for the long running token generation task */
config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
Firebase.begin(&config, &auth);
Firebase.reconnectWiFi(true);
json.set("id:","1");

return;}

//......................................................................//

void loop() {
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);
x0 
= RAD_TO_DEG * (atan2(-a.acceleration.y, -a.acceleration.z)+PI); // value of a
   if (x0 > 180) x0 = 360 - x0; // To make sure that the angles start from zero degrees
                               // (according to periodicity of cosines)
yo0 = RAD_TO_DEG * (atan2(-a.acceleration.x, a.acceleration.z)+PI); // value of b
z0 = RAD_TO_DEG * (atan2(-a.acceleration.y, -a.acceleration.x)+PI);// value of c

w = abs(180 - (x0 + x1));
c = sqrt(pow(20,2) + pow(40,2) - 20*40*(cos(w))); //The Law of cosines. (a^2+b^2-a*b*cos(w))

json.set("/alng0/x0", x0); // adding data to json -> json format: [key , val]
Serial.println (x0);  
json.set("w/w1+0" , w);
json.set("c" , c);
Serial.println ("end transmiting to firebase data base! "); // To inform about the completion of reading and sending data  
Firebase.RTDB.set(&fbdo, F("json"), &json); // sending the json to the data base 
delay(1000);}