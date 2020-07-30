
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <esp_now.h>
#include "AzureIotHub.h"
#include "Esp32MQTTClient.h"
#include "time.h"

/*****************************************
** Constants and global-scope variables **
*****************************************/

#define INTERVAL 150000 //5 minutes interval
#define DEVICE_ID "Device_825"
#define MESSAGE_MAX_LEN 256
uint8_t broadcastAddress[] = {0xFC, 0xF5, 0xC4, 0x2F, 0x88, 0x84}; // REPLACE THIS WITH YOUR SLAVE ESP MAC ADDRESS




/*****************************************
** ESP-NOW configuration **
*****************************************/



/*
Data Sending
*/


// Structure to send data
typedef struct struct_message {
  float a;
  float b;
  bool d;
} struct_message;


// Variable to store if sending data was successful
String success;


// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


// struct_message to send
struct_message myData;







/*
Data Recieving
*/

//Structure example to recieve data
typedef struct struct_rcv {
    float temp;
} struct_rcv;

// to recieve
float RoomTemperature = 300;


// Create a struct_message called myData for recieving
struct_rcv myRecieved;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myRecieved, incomingData, sizeof(myRecieved));

  RoomTemperature = myRecieved.temp;
  
  // write to Serial
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Room Temp:");
  Serial.println(myRecieved.temp);
  
  
}









/*****************************************
** WIFI and azure configuration **
*****************************************/



// SSID and password of WiFi
const char* ssid     = "RouterSpek2.4";
const char* password = "0526535565";

static const char* connectionString ="HostName=IEMIotHub1.azure-devices.net;DeviceId=azuredevice8;SharedAccessKey=RP4aC/guvrIZo6UurI9ltX6/jyXH4KMyWk4NXCI+ixs=";

const char *messageData1 = "{\"deviceId\":\"%s\", \"Time\":\"%s\", \"Temperature\":%f, \"DoorOpen\":%d, \"RoomTemperature\":%f, \"Humidity\":%f}";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;


static bool hasWifi = false;
static bool messageSending = true;
static uint64_t send_interval_ms;
static uint64_t espnow_send_interval_ms;

char timeStringBuff[50]; //50 chars should be enough



/*************
** Utilities **
*************/


static void InitWifi()
{
  Serial.println("Connecting...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  hasWifi = true;
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}



static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    Serial.println("Send Confirmation Callback finished.");
  }
}



static void MessageCallback(const char* payLoad, int size)
{
  Serial.println("Message callback:");
  Serial.println(payLoad);
}



static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}



static int  DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
  LogInfo("Try to invoke method %s", methodName);
  const char *responseMessage = "\"Successfully invoke device method\"";
  int result = 200;

  if (strcmp(methodName, "start") == 0)
  {
    LogInfo("Start sending temperature and humidity data");
    messageSending = true;
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    LogInfo("Stop sending temperature and humidity data");
    messageSending = false;
  }
  else
  {
    LogInfo("No method %s found", methodName);
    responseMessage = "\"No method found\"";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}



void updateLocalTime()
{
  time_t rawtime;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.print("Function time: ");

}



/*****************************************
** BME280 and light sensor configuration **
*****************************************/

Adafruit_BME280 bme; // I2C
float V;
const int lightSensot = 35;
int counter = 0;


float getTemp() {
    return bme.readTemperature();
}

float getHumidity() {
    return bme.readHumidity();
}

bool getDoorStatus(){
    V = ReadVoltage(lightSensot); //vn
    if (V > 2.1 ) { //2.3
      return 0;
    } else {
      return 1;
    }
}

double ReadVoltage(byte pin) { // read voltage from light sensor
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {
    Serial.begin(115200);


    /*****************************************
    ** BME280 sensor setup **
    *****************************************/


    // initializing BME280 sensor
    while(!Serial);    // time to get serial running
    Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }



    /*****************************************
    ** WIFI setup and test **
    *****************************************/

    Serial.println("-- Default Test --");
    Serial.println();
  
    // Initialize the WiFi module
    Serial.println(" > WiFi");
    hasWifi = false;
    InitWifi();
    if (!hasWifi)
    {
      return;
    }
    randomSeed(analogRead(0));
  
    Serial.println(" > IoT Hub");
    Esp32MQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "GetStarted");
    Esp32MQTTClient_Init((const uint8_t*)connectionString, true);
  
    Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
    Esp32MQTTClient_SetMessageCallback(MessageCallback);
    Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
    Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  
    send_interval_ms = millis();
    espnow_send_interval_ms = millis();


    /*****************************************
    ** ESP-NOW setup **
    *****************************************/

      // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

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

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);


}

void loop() {


  float temperature = getTemp();
  float Humidity = getHumidity();
  bool doorOpen = getDoorStatus();

  if( (int)(millis() - espnow_send_interval_ms) > 500)
  {

    // Set values to send
    myData.a = temperature;
    myData.b = Humidity;
    myData.d = doorOpen;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
     
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }

    espnow_send_interval_ms = millis();

  }



  if (hasWifi){
    if ((messageSending && doorOpen && (int)(millis() - send_interval_ms) > 20000 ) || (messageSending && (int)(millis() - send_interval_ms) >= INTERVAL))
    {
      // Send teperature data
      if(doorOpen){
        counter = 0;
       }

      char messagePayload[MESSAGE_MAX_LEN];
      updateLocalTime();

      
      snprintf(messagePayload, MESSAGE_MAX_LEN, messageData1, DEVICE_ID, timeStringBuff , temperature, doorOpen, RoomTemperature, Humidity);
      Serial.println(messagePayload);
      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(messagePayload, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);
  
      send_interval_ms = millis();
    }
//    else
//    {
//      Esp32MQTTClient_Check();
//    }
  }
  delay(100);

}
