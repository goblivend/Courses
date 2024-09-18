#include <SoftwareSerial.h>
#include "HCSR04.h"
#include "DHT11.h"

SoftwareSerial e5(6, 7);                    // (RX, TX)
UltraSonicDistanceSensor distSensor(4, 5);  // Trig, Echo
DHT11 dht11(3);


void join() {
  static char *APP_EUI = "0000000000000000";
  static char *DEV_EUI = "70B3D57ED0068382";
  static char *APP_KEY = "272F54BA747701C731AF1DEFAC62FED3";

  Serial.println("Setting DevEUI!");
  e5.write("AT+ID=DevEUI,");
  e5.write(DEV_EUI);
  e5.write("\n");
  e5.flush();
  delay(350);
  Serial.println("Setting AppEUI!");
  e5.write("AT+ID=AppEUI,");
  e5.write(APP_EUI);
  e5.write("\n");
  e5.flush();
  delay(350);
  Serial.println("Setting AppKey!");
  e5.write("AT+KEY=APPKEY,");
  e5.write(APP_KEY);
  e5.write("\n");
  e5.flush();
  delay(350);
}

int temperature = 0;
int humidity = 17;
int distance = 42;
unsigned long prev = 0;
unsigned long interval = 3000;
unsigned long sendPrev = 0;
unsigned long sendInterval = 10000;


void send_message(String msg) {
  msg = String("AT+MSG=") + msg + String("\n");

  e5.write(msg.c_str());
  Serial.write(msg.c_str());
}

void send_data() {
  String msg = String("");
  if (temperature < 10)
    msg += String(0);
  msg += String(temperature);
  if (humidity < 10)
    msg += String(0);
  msg += String(humidity);
  if (distance < 10)
    msg += String(0);
  msg += String(distance);

  send_message(msg);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  e5.begin(9600);

  delay(2000);
  Serial.println("Starting Setup!");
  // join();
  e5.println("AT+JOIN");

  delay(5000);
  Serial.println("TETE");
  send_data();
}

void loop() {
  while (Serial.available() > 0) {
    e5.write(Serial.read());
  }

  String message = String();
  while (e5.available() > 0) {
    message += e5.readString();
  }
  // D7 DB DF E3 E7 --------------hgfhgfhgfhgfhgf---------------

  if (message.length() > 0) {
    Serial.println("====");
    Serial.print(message);
    Serial.println("====");
    // +MSG: PORT: 15; RX: "D7"
    
    if (strstr(message.c_str(), "+MSG: PORT")) {
      Serial.println("MESSAGE");
      String speed = message.substring(message.indexOf("+MSG: PORT") + 21, message.indexOf("+MSG: PORT") + 23);
      // Serial.println(speed);
      if (speed.equals("D7")) {
        sendInterval = 10000;
      } else if (speed.equals("DB")) {
        sendInterval = 20000;
      } else if (speed.equals("DF")) {
        sendInterval = 30000;
      } else if (speed.equals("E3")) {
        sendInterval = 40000;
      } else if (speed.equals("E7")) {
        sendInterval = 50000;
      }
      Serial.print("Sending every: ");
      Serial.println(sendInterval / 1000);
    }
  }
  unsigned long curr = millis();
  if (curr >= prev + interval) {
    prev = curr;
    distance = distSensor.measureDistanceCm();
    Serial.println(distance);
    int result = dht11.readTemperatureHumidity(temperature, humidity);

    if (result == 0) {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C\tHumidity: ");
      Serial.print(humidity);
      Serial.println(" %");
    } else {
      // Print error message based on the error code.
      Serial.println(DHT11::getErrorString(result));
    }
  }
  if (curr >= sendPrev + sendInterval) {
    sendPrev = curr;
    send_data();
  }
}