/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <PCD8544.h>
#define BL 27
#include <Tone32.h>
#include "time.h"

#define BUZZER_PIN 22
#define BUZZER_CHANNEL 0
uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0xFF, 0x6E, 0xA0}; // REPLACE THIS WITH YOUR MASTER ESP MAC ADDRESS

static const byte glyph[] = { B00010000, B00110100, B00110000, B00110100, B00010000 };
static PCD8544 lcd;

static uint64_t espnow_send_interval_ms;
static uint64_t door_open_interval_ms;

bool door = 0;
bool door_open_flag = 0;
float RoomTemperature = 0;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float a;
  float b;
  bool d;
} struct_message;


// Create a struct_message called myData
struct_message myData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  door = myData.d;
  
  // write to Serial
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Fridge Temp:");
  Serial.println(myData.a);
  Serial.print("Humidity:");
  Serial.println(myData.b);
  Serial.print("door:");
  Serial.println(myData.d);
  Serial.println();

  // write to LCD
  lcd.setCursor(0, 0);
  lcd.print("Fridge Temp:");
  lcd.setCursor(0, 1);
  lcd.print(myData.a);
  lcd.setCursor(0, 2);
  lcd.print("Humidity:");
  lcd.setCursor(0, 3);
  lcd.print(myData.b);
  lcd.setCursor(0, 4);
  lcd.print("Room Temp:");
  lcd.setCursor(0, 5);
  lcd.print(RoomTemperature);
  //lcd.write(' ');
  //lcd.write(0);  // write the smiley
}


/*
Data Sending
*/



//Structure example to send data
//Must match the receiver structure
typedef struct struct_send {
    float temp;
} struct_send;


// Variable to store if sending data was successful
String success;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// struct_message to send
struct_send toSend;



/*
Room temperature reading
*/


const int TMP36 = 32;


double ReadVoltage(byte pin) { // read voltage from light sensor
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
}


double getRoomTemp(byte pin) {
  double voltage = ReadVoltage(pin);
  return (voltage - 0.5) * 100 - 3;
}



 /*
Setup
*/



void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

//  pinMode(speakerPin, OUTPUT);

  // LCD setup
  lcd.begin(84, 48);
  lcd.createChar(0, glyph);
  pinMode(BL, OUTPUT);
  digitalWrite(BL, HIGH);


  //////// Init ESP-NOW ///////////

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
   
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }


  /*  ESP-NOW recieve setup  */
  

    
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);



  /*  ESP-NOW send setup  */


  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  
    // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
    
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  RoomTemperature = getRoomTemp(TMP36);
  espnow_send_interval_ms = millis();
  door_open_interval_ms = millis();
}
 
void loop() {
  
  RoomTemperature = getRoomTemp(TMP36);
  
  if( (int)(millis() - espnow_send_interval_ms) > 10000)
  {

    // Set values to send
    toSend.temp = RoomTemperature;
 
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &toSend, sizeof(toSend));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    espnow_send_interval_ms = millis();

  }


  // Door alarm
  if( (int)(millis() - door_open_interval_ms) > 3000)
  {
    door_open_flag = 1;
  }



  if(!door)
  {
    door_open_interval_ms = millis();
    door_open_flag = 0;
  }



  if(door_open_flag)
  {
    tone(BUZZER_PIN, NOTE_C4, 500, BUZZER_CHANNEL);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    tone(BUZZER_PIN, NOTE_D4, 500, BUZZER_CHANNEL);
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
   }
   
  delay(100);
  
}
