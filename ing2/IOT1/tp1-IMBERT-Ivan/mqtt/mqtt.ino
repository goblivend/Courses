/*
  ArduinoMqttClient - WiFi Advanced Callback

  This example connects to a MQTT broker and subscribes to a single topic,
  it also publishes a message to another topic every 10 seconds.
  When a message is received it prints the message to the Serial Monitor,
  it uses the callback functionality of the library.

  It also demonstrates how to set the will message, get/set QoS, 
  duplicate and retain values of messages.

  The circuit:
  - Arduino MKR 1000, MKR 1010 or Uno WiFi Rev2 board

  This example code is in the public domain.
*/

#include <ArduinoMqttClient.h>
#include <WiFi.h>

#include "DHT11.h"

#define DHTPIN 15
#define LED_BUILTIN 2

const char* ssid     = "NETGEAR44";
const char* password = "royalvase693";
const char* broker   = "192.168.1.22";
int         port     = 1883;


// To connect with SSL/TLS:
// 1) Change WiFiClient to WiFiSSLClient.
// 2) Change port value from 1883 to 8883.
// 3) Change broker value to a server with a known SSL/TLS root certificate 
//    flashed in the WiFi module.

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
DHT11 dht(DHTPIN);

const char temperatureTopic[]  = "test/esp32/dht/temperature";
const char humidityTopic[]     = "test/esp32/dht/humidity";
const char ledToggleTopic[]    = "test/esp32/led/toggle";
const char ledStatusTopic[]    = "test/esp32/led/status";

float temp = 0;
float hum = 0;

const long interval = 500;
unsigned long previousMillis = 0;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

    // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(ledToggleTopic);
  Serial.println();

  // subscribe to a topic
  // the second parameter sets the QoS of the subscription,
  // the the library supports subscribing at QoS 0, 1, or 2
  int subscribeQos = 1;

  mqttClient.subscribe(ledToggleTopic, subscribeQos);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(inTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(ledToggleTopic);
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    String payloadH;
    String payloadT;

    payloadH += dht.readHumidity();
    payloadT += dht.readTemperature();


    bool retained = false;
    int qos = 1;
    bool dup = false;

    Serial.print("Sending message to topic: ");
    Serial.println(temperatureTopic);
    Serial.println(payloadT);
    mqttClient.beginMessage(temperatureTopic, payloadT.length(), retained, qos, dup);
    mqttClient.print(payloadT);
    mqttClient.endMessage();

    Serial.print("Sending message to topic: ");
    Serial.println(humidityTopic);
    Serial.println(payloadH);
    mqttClient.beginMessage(humidityTopic, payloadH.length(), retained, qos, dup);
    mqttClient.print(payloadH);
    mqttClient.endMessage();

    Serial.println();
  }
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.print("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', duplicate = ");
  Serial.print(mqttClient.messageDup() ? "true" : "false");
  Serial.print(", QoS = ");
  Serial.print(mqttClient.messageQoS());
  Serial.print(", retained = ");
  Serial.print(mqttClient.messageRetain() ? "true" : "false");
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");

  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
  }
  Serial.println();

  Serial.println();

  static int state = LOW;
  state = !state;

  digitalWrite(LED_BUILTIN, state);

  String payloadLed;
  payloadLed += state;

  bool retained = false;
  int qos = 1;
  bool dup = false;

  Serial.print("Sending message to topic: ");
  Serial.println(ledStatusTopic);
  Serial.println(payloadLed);
  mqttClient.beginMessage(ledStatusTopic, payloadLed.length(), retained, qos, dup);
  mqttClient.print(payloadLed);
  mqttClient.endMessage();
}
