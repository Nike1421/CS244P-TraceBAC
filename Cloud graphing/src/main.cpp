#include <Arduino.h>
#include <HttpClient.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>
#include <Adafruit_AHTX0.h>

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

// Make sure to add app.py and static/index.html to EC2 instance 
//  Make static folder if needed

// This example downloads the URL "http://arduino.cc/"
char ssid[50]; // your network SSID (name)
char pass[50]; // your network password (use for WPA, or use as key for WEP)

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30 * 1000;

// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;

Adafruit_AHTX0 aht;

float humidity_val, temperature_val;

// Simulated sensor values for BPM, BAC, and accelerometer (replace with actual sensors)
int bpm = 95;
float bac = 0.05;
float accelX = 1.2, accelY = -0.3, accelZ = 0.5;

void nvs_access()
{
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }

  ESP_ERROR_CHECK(err);

  // Open
  Serial.printf("\n");
  Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");

  nvs_handle_t my_handle;

  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  }
  else
  {
    Serial.printf("Done\n");
    Serial.printf("Retrieving SSID/PASSWD\n");
    size_t ssid_len;
    size_t pass_len;
    err = nvs_get_str(my_handle, "ssid", ssid, &ssid_len);
    err |= nvs_get_str(my_handle, "pass", pass, &pass_len);
    switch (err)
    {
    case ESP_OK:
      Serial.printf("Done\n");
      Serial.printf("SSID = %s\n", ssid);
      Serial.printf("PASSWD = %s\n", pass);
      break;
    case ESP_ERR_NVS_NOT_FOUND:
      Serial.printf("The value is not initialized yet!\n");
      break;
    default:
      Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
  }
  // Close
  nvs_close(my_handle);
}

void sensor_setup()
{
  if (!aht.begin())
  {
    Serial.println("Could not find AHT? Check wiring");
    while (1)
      ;
  }
  Serial.println("AHT10 or AHT20 found");
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  // Retrieve SSID/PASSWD from flash before anything else
  nvs_access();

  // We start by connecting to a WiFi network
  // delay(1000);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("MAC address: ");
  Serial.println(WiFi.macAddress());

  // sensor_setup();
}


void loop()
{
  int err = 0;
  WiFiClient c;
  HttpClient http(c);
  bpm = random(60, 120);  // Random BPM between 60 and 120
  bac = random(0, 100) / 100.0;  // Random BAC between 0.0 and 1.0
  accelX = random(-10, 10) / 10.0;  // Random Accel X between -1.0 and 1.0
  accelY = random(-10, 10) / 10.0;  // Random Accel Y between -1.0 and 1.0
  accelZ = random(-10, 10) / 10.0;  // Random Accel Z between -1.0 and 1.0
  String postData ="{\"bpm\": " + String(bpm) + ", \"bac\": " + String(bac) + 
                   ", \"accel_x\": " + String(accelX) + ", \"accel_y\": " + String(accelY) + 
                   ", \"accel_z\": " + String(accelZ) + "}";
  // Send POST request
  Serial.print(postData);
  Serial.print("");

  http.beginRequest();
  // 18.117.11.134 - Tiffany EC2 instance
  // 18.216.69.210 - Om EC2 instance
  err = http.post("18.117.11.134", 5000, "api/data", NULL);
  if (err == 0)
  {
    Serial.println("POST request started successfully");

    // Add headers manually after starting the request
    http.sendHeader("Content-Type", "application/json");
    http.sendHeader("Content-Length", postData.length());
    // http.beginBody();
    http.print(postData);  // Send the JSON data in the body

    // Serial.println("startedRequest ok");
    err = http.responseStatusCode();
    if (err >= 0)
    {
      Serial.print("Got status code: ");
      Serial.println(err);

      // Usually you'd check that the response code is 200 or a
      // similar "success" code (200-299) before carrying on,
      // but we'll print out whatever response we get

      err = http.skipResponseHeaders();

      if (err >= 0)
      {
        int bodyLen = http.contentLength();
        Serial.print("Content length is: ");
        Serial.println(bodyLen);
        Serial.println();
        Serial.println("Body returned follows:");

        // Now we've got to the body, so we can print it out
        unsigned long timeoutStart = millis();
        char c;

        // Whilst we haven't timed out & haven't reached the end of the body
        while ((http.connected() || http.available()) &&
               ((millis() - timeoutStart) < kNetworkTimeout))
        {
          if (http.available())
          {
            c = http.read();
            // Print out this character
            Serial.print(c);
            bodyLen--;
            // We read something, reset the timeout counter
            timeoutStart = millis();
          }
          else
          {
            // We haven't got any data, so let's pause to allow some to
            // arrive
            delay(kNetworkDelay);
          }
        }
      }
      else
      {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }
    }
    else
    {
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  }
  else
  {
    Serial.print("Connect failed: ");
    Serial.println(err);
  }
  http.stop();
  // And just stop, now that we've tried a download
  delay(2000);
}