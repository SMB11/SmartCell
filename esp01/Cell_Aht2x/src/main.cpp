#include <Arduino.h>

#include <Adafruit_AHTX0.h>
 #ifdef ESP8266
#include <ESP8266WiFi.h> // Pins for board ESP8266 Wemos-NodeMCU
#else
#include <WiFi.h>
#include <SPIFFS.h>
#endif
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

Adafruit_AHTX0 aht;
 
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
unsigned long lastSensorUpdateTime;
int sensorUpdateInterval = 15000;

double tempCurrentValue;
double humidityCurrentValue;

const char *ssid = "TechZone";
const char *password = "*******";



// const char *mqtt_server = "192.168.0.104";
const char *mqtt_server = "192.168.0.103";

//const char *cellPublishTopic = "MeteoSensor";

const char *cellPublishTopic = "InsideSensor2";
const char *errorTopic = "Errors";
const int mqtt_port = 1883;

int timeInMinutesNow;

StaticJsonDocument<96> doc;
StaticJsonDocument<96> errorDoc;
WiFiClient espClient;
PubSubClient client(espClient);
// #include <task.h>

void updateTime(){
    if (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  timeInMinutesNow = timeClient.getHours()*60+timeClient.getMinutes();
}

void timeInit()
{
    timeClient.begin();
    timeClient.setTimeOffset(14400);
    updateTime();
}
void publish_sensorData()
{
    updateTime();
    delay(300);
    doc["timeInMinutesNow"] = timeInMinutesNow;
    // doc["time"] = 123;
    doc["insideTemp2"] = tempCurrentValue;
    doc["insideHumidity"] = humidityCurrentValue;
    delay(1);
    client.beginPublish(cellPublishTopic, measureJson(doc), true);
    BufferingPrint bufferedClient(client, 32);
    serializeJson(doc, bufferedClient);
    bufferedClient.flush();
    client.endPublish();
}

void pubslishError(const char *errType, const char *error)
{
    errorDoc["time"] = millis();
    errorDoc["controller"] = "InsideSensor2";
    errorDoc["type"] = errType;
    errorDoc["error"] = error;

    client.beginPublish(errorTopic, measureJson(errorDoc), true);
    BufferingPrint bufferedClient(client, 32);
    serializeJson(errorDoc, bufferedClient);
    bufferedClient.flush();
    client.endPublish();
}

void setup_wifi()
{
    delay(10);
    Serial.print("\nConnecting to ");
    Serial.println(ssid);

    // WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid,password);

    // WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("\nWiFi connected\nIP address: ");
    Serial.println(WiFi.localIP());
}
void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
//        String clientId = "MeteoCell";
        String clientId = "InsideSensor2";

        if (client.connect(clientId.c_str()))
        {
            const char *type = "mqtt";
            const char *status = "reconnected";
            Serial.println(status);
            pubslishError(type, status);
            // client.subscribe(setTime_topic); // subscribe the topics here
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds"); // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}
void callback(char *topic, byte *payload, unsigned int length)
{
    String incommingMessage = "";
    for (int i = 0; i < length; i++)
        incommingMessage += (char)payload[i];

    Serial.println("Message arrived [" + String(topic) + "]" + incommingMessage);

    // //--- check the incomming message
    // if (strcmp(topic, servoAngleTopic) == 0)
    // {
    //     // Serial.println("Message received watering_tasks ");
    //     int angle = incommingMessage.toInt();
    //     Serial.println(angle );
    //     servo1.write(angle);
    //      client.publish(currentServoAngleTopic,(char *)currentServoAngle);

    // }
}

void mqtt_loop()
{
    client.loop();
    if (!client.connected())
        reconnect(); // TODO: change reconnect function
}

void mqtt_init()
{
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    client.setBufferSize(1000);
}



void getTempAndHumidityData()
{

    sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  double temperature = temp.temperature;

    if (temperature > 60 || temperature < -25)
    {
        const char *errType = "invalid Temp";
        static char outstr[15];
        dtostrf(temperature, 7, 3, outstr);
        pubslishError(errType, outstr);
        // getTempAndHumidityData();
    }
    else
    {
        if (abs(temperature - tempCurrentValue) < 50)
        {
            tempCurrentValue = temperature;
        }
    }

    int hum = humidity.relative_humidity;

    humidityCurrentValue = hum;
}

 /////////
void setup() 
{
  Serial.begin(115200);
  
  timeInit();
    setup_wifi();
    mqtt_init();
    Serial.println("Adafruit AHT10/AHT20 demo!");
 
  if (! aht.begin()) 
  {
    Serial.println("Could not find AHT? Check wiring");
    // while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");
}
 
void loop() {
  
   mqtt_loop();
    unsigned long now = millis();
    if (now - lastSensorUpdateTime > sensorUpdateInterval)
    {
        lastSensorUpdateTime = now;
        getTempAndHumidityData();
        publish_sensorData();

        Serial.println(tempCurrentValue);
        Serial.println(humidityCurrentValue);
    }
}