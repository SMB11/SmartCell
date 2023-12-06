#include <Arduino.h>

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

#include <Wire.h>
#include <AHTxx.h>

float ahtValue; // to store T/RH result

AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR); // sensor address, sensor type

WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP);
NTPClient timeClient(ntpUDP, "us.pool.ntp.org");

unsigned long lastSensorUpdateTime;
int sensorUpdateInterval = 15000;

unsigned long lastMQTTReconnectAttempt;
int mqttUpdateInterval= 5000;

double tempCurrentValue;
double humidityCurrentValue;

const char *ssid = "TechZone";
const char *password = "*******";



const char *mqtt_server = "192.168.0.106";
// const char *mqtt_server = "192.168.0.103";

// const char *cellPublishTopic = "MeteoSensor";

const char *cellPublishTopic = "InsideSensor2";
const char *errorTopic = "Errors";
const int mqtt_port = 1883;

int timeInMinutesNow;

WiFiClient espClient;
PubSubClient client(espClient);
// #include <task.h>

void updateTime()
{
  if (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  timeInMinutesNow = timeClient.getHours() * 60 + timeClient.getMinutes();
}

void timeInit()
{
  timeClient.begin();
  timeClient.setTimeOffset(14400);
  updateTime();
}
void publish_sensorData()
{

  StaticJsonDocument<96> doc;
  updateTime();
  delay(300);
  doc["timeInMinutesNow"] = timeInMinutesNow;
  // doc["time"] = 123;
  // doc["outsideTemp"] = tempCurrentValue;
  doc["insideTemp"] = tempCurrentValue;
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

  StaticJsonDocument<96> errorDoc;

  errorDoc["time"] = millis();
  // errorDoc["controller"] = "MeteoSensor";
  errorDoc["controller"] = "insideSensor2";
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
  WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
  lastMQTTReconnectAttempt = 0;
}
bool reconnect()
{
 
    Serial.print("Attempting MQTT connection...");
    //        String clientId = "MeteoCell";
    // String clientId = "MeteoSensor";
    String clientId = "InsideSensor2";


    if (client.connect(clientId.c_str()))
    {
      const char *type = "mqtt";
      const char *status = "reconnected";
      Serial.println(status);
      pubslishError(type, status);
      // client.subscribe(setTime_topic); // subscribe the topics here
    }
    // else
    // {
    //   Serial.print("failed, rc=");
    //   Serial.print(client.state());
    //   Serial.println(" try again in 5 seconds"); // Wait 5 seconds before retrying
    //   delay(5000);
    // }
    return client.connected();
  
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
  // client.loop();
  // if (!client.connected())
  //   reconnect(); // TODO: change reconnect function
if (!client.connected())
    {
        long now = millis();
        if (now - lastMQTTReconnectAttempt > mqttUpdateInterval)
        {
            lastMQTTReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
            {
                lastMQTTReconnectAttempt = 0;
            }
        }
    }
    else
    {
        // Client connected

        client.loop();
    }
}

void mqtt_init()
{
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(1000);
}

void getTempAndHumidityData()
{

  double temperature = aht20.readTemperature();

  if (ahtValue != AHTXX_ERROR) // AHTXX_ERROR = 255, library returns 255 if error occurs
  {
    tempCurrentValue = temperature;
  }
  else
  {
    const char *errType = "invalid Temp";
    static char outstr[15];
    dtostrf(temperature, 7, 3, outstr);
    pubslishError(errType, outstr);
    if (aht20.softReset() == true)
      Serial.println(F("reset success")); // as the last chance to make it alive
    else
      Serial.println(F("reset failed"));
  }

  int hum = aht20.readHumidity();
  if (hum > 100 && hum < 200)
  {
    humidityCurrentValue = 100;
  }
  else
  {
    humidityCurrentValue = hum;
  }
}

/////////
void setup()
{
  Serial.begin(115200);

  timeInit();
  setup_wifi();
  mqtt_init();
  // Serial.println("Adafruit AHT10/AHT20 demo!");

  // if (! aht.begin())
  // {
  //   Serial.println("Could not find AHT? Check wiring");
  //   // while (1) delay(10);
  // }
  aht20.begin(0, 2);
  // Serial.println("AHT10 or AHT20 found");
}

void loop()
{

  mqtt_loop();
  unsigned long now = millis();
  if (now - lastSensorUpdateTime > sensorUpdateInterval)
  {
    lastSensorUpdateTime = now;
    getTempAndHumidityData();
    publish_sensorData();

    // Serial.println(tempCurrentValue);
    // Serial.println(humidityCurrentValue);
  }
}