#include <ESP8266WiFi.h>
#include <DHT.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "SolarVFS"
#define WIFI_PASSWORD "Maju@2022"
// Insert Firebase project API Key
#define API_KEY "AIzaSyAJ6XUW90uW5ud0Cf3DceczE-4kBCQuvGg"
// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "admin1@svf.org"
#define USER_PASSWORD "maju@1234"
// Insert RTDB URLefine the RTDB URL
#define DATABASE_URL "https://solarverticalfarming-default-rtdb.firebaseio.com/"

// Pin GPIO Definition
#define DHTPIN D1          // GPIO Pin where the dht11 is connected
DHT dht = DHT(DHTPIN, DHT11 , 6);
uint8_t  moisturePin = A0;             // moisteure sensor pin
uint8_t  fanRelay = D2;              // Fan Relay Signal Pin
uint8_t  motorRelay = D3;            // Motor Pump Relay Signal Pin
uint8_t  Red = D5 ;                  // 
uint8_t  Green = D6 ;
uint8_t  Blue = D7;

//Conditional Programming
bool fanCon = false;
bool motorCon = false;
int rgbState = 0;
int Tlimit = 27;
int Mlimit = 50 ; 

// Define Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save USER UID
String uid;

// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
String tempPath = "/temperature";
String humPath = "/humidity";
String moisPath = "/moisture";
String timePath = "/timestamp";

// Parent Node (to be updated in every loop)
String parentPath;

FirebaseJson json;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Variable to save current epoch time
int timestamp;
// Variable to save sensors reading
float temperature;
float humidity;
float moisture;
// Timer variables (send new readings every 1 minutes)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 60000;

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
}

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}



void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(fanRelay , OUTPUT);
  pinMode(motorRelay , OUTPUT);
  pinMode(Red , OUTPUT);
  pinMode(Green , OUTPUT);
  pinMode(Blue , OUTPUT);
  initWiFi();
  timeClient.begin();
  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the user sign in credentials
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  // Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  // Assign the maximum retry of token generation
  config.max_token_generation_retry = 5;

  // Initialize the library with the Firebase authen and config
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  // Update database path
  databasePath = "/UsersData/" + uid + "/readings";

}

void loop() {
  readsensor();
  // Send new readings to database
  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    //Get current timestamp
    timestamp = getTime();
    Serial.print ("time: ");
    Serial.println (timestamp);

    parentPath= databasePath + "/" + String(timestamp);

    json.set(tempPath.c_str(), String(temperature));
    json.set(humPath.c_str(), String(humidity));
    json.set(moisPath.c_str(), String(moisture));
    json.set(timePath, String(timestamp));
    Serial.print("Set json... ");
    Serial.print( Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
    Serial.println("------SENT------");
    if (rgbState == 0) {
      RGBcontroller(250,250,250); //white
      rgbState = 1;
      Serial.println("White Color");
    }
    else if (rgbState == 1) {
      RGBcontroller(250,0,0); //red
      rgbState = 2;
      Serial.println("Red Color");
    }
    else if (rgbState == 2) {
      RGBcontroller(0,250,0); //green;
      rgbState = 3;
      Serial.println("Green Color");
    }
    else if (rgbState == 3) {
      RGBcontroller(0,0,250); //blue
      rgbState = 0;
      Serial.println("Blue Color");
    }
  }
  checkFan(temperature , Tlimit);
  checkMotor(moisture , Mlimit);
  

}

void readsensor(){
  temperature = dht.readTemperature();     // read temperature;
  humidity = dht.readHumidity();     // read humiduty;
  
  if (isnan(temperature) || isnan(humidity))

  {

    Serial.println("Failed to read from DHT sensor!");
    delay(3000);

    return;

  }
  moisture =  ( 100.00 - ( (analogRead(moisturePin) / 1023.00) * 100.00 ) );
    Serial.println("--------- Sensors Readings -----------");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("");
  Serial.print("Moisture: ");
  Serial.print(moisture);
  Serial.println("");
  delay(500);
}

void checkFan(float temperatureF,float limitTempt){
  if (temperatureF >= limitTempt){
    if (fanCon == false){
    digitalWrite(fanRelay , HIGH);
    fanCon = true;
    Serial.println("Fan ON"); 
    }    
  }
  else{
    if (fanCon == true){
      digitalWrite(fanRelay , LOW);
      fanCon = false;
      Serial.println("Fan OFF");
    }
  }
}

void checkMotor(float moistureF , float limitMois){
  if(moistureF >= limitMois){
    if (motorCon == false) {
    digitalWrite(motorRelay , HIGH);
    motorCon = true;
    Serial.println("Motor Pump ON"); 
    }    
  }
  else{
    if (motorCon == true){
    digitalWrite(motorRelay , LOW);
      motorCon = false;
      Serial.println("Motor Pump OFF");
    }
  }
}

void RGBcontroller( int R , int G , int B){
  if (R >= 0 && R <= 255){
    analogWrite(Red , R);
  }
  if (G >= 0 && G <= 255){
    analogWrite(Green , G);
  }
  if (B >= 0 && B <= 255){
    analogWrite(Blue , B);
  }
}
