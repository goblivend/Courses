/*
  This a simple example of the aREST Library for the ESP32 WiFi chip.
  See the README file for more details.

  Written in 2017 by Marco Schwartz under a GPL license.
  
*/

// Import required libraries
#include <WiFi.h>
#include <aREST.h>

#include "DHT11.h"

// Create aREST instance
aREST rest = aREST();

#define DHTPIN 13
#define LED_BUILTIN 2

DHT11 dht11(DHTPIN);

// WiFi parameters
const char* ssid     = "Partages";
const char* password = "Partages";

// Create an instance of the server
WiFiServer server(80);

// Variables to be exposed to the API
int temperature = 0;
int humidity = 0;

// Auxiliary timer variables
unsigned long prev = 0;
const long delta = 5000;

// Custom function accessible by the API
int ledControl(String command) {
    static int state = LOW;
    state = !state;

    digitalWrite(LED_BUILTIN, state);
    return 1;
}

void setup()
{
    // Start Serial
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Expose variables to REST API
    temperature = 0;
    humidity = 0;
    rest.variable("temperature",&temperature);
    rest.variable("humidity",&humidity);

    // Function to be exposed
    rest.function("led",ledControl);;

    // Give name & ID to the device (ID should be 6 characters long)
    rest.set_id("1");
    rest.set_name("esp32");

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    // Start the server
    server.begin();
    Serial.println("Server started");

    // Print the IP address
    Serial.println(WiFi.localIP());
}

void loop()
{
    unsigned long curr = millis();
    if (curr - prev >= delta) {
        prev = curr;

        // Attempt to read the temperature and humidity values from the DHT11 sensor.
        int result = dht11.readTemperatureHumidity(temperature, humidity);

        // Check the results of the readings.
        // If the reading is successful, print the temperature and humidity values.
        // If there are errors, print the appropriate error messages.
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

    // Handle REST calls
    WiFiClient client = server.available();
    if (!client) {
        return;
    }
    // Serial.println("Got client");
    while(!client.available()){
        delay(1);
    }
    // Serial.println("Sending Headers");
    rest.send_http_headers();
    // Serial.println("Sending res");
    rest.handle(client);
}
