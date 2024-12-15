#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <HttpClient.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>
#include <PulseSensorPlayground.h>
#include <PulseSensorPlayground.h>

#define USE_ARDUINO_INTERRUPTS false // Set-up low-level interrupts for most accurate BPM math.

// Sensor Pins
#define BUZZER 26
#define RED_LED 25
#define ALC_SENSOR 33
#define PULSE_SENSOR 32
#define BUZZER_CHANNEL 0

// Sensor Thresholds
#define BAC_THRESHOLD 3
#define FALL_THRESHOLD 300
#define PULSE_SENSOR_THRESHOLD 550

#define SAMPLE_INTERVAL 10
#define BEEP_DURATION 5000

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pulse Sensor Object
PulseSensorPlayground pulseSensor;

// Pulse Sensor Variables
int my_bpm = 0;
int pulse_analog_signal = 0;

// Alcohol Sensor Variables
float adcValue = 0, val = 0, mgL = 0;

// Accelerometer Object
MPU6050 fallDetectorMPU(0x69);

// Accelerometer Variables
int currentIndex = 0;
float jerkMagnitude = 0;
float jerkValues[10] = {0};
float prevAccX = 0, prevAccY = 0, prevAccZ = 0;

unsigned long button_time = 0;
bool buttonInterrupted = false;
unsigned long delayStartTime = 0;
unsigned long last_button_time = 0;

struct Button
{
	const uint8_t PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};

Button fallCheckButton = {5, 0, false};

void IRAM_ATTR isr()
{
	button_time = millis();
	if (button_time - last_button_time > 250)
	{
		fallCheckButton.numberKeyPresses++;
		fallCheckButton.pressed = true;
		last_button_time = button_time;
		buttonInterrupted = true;
	}
}

// WIFI Variables
WiFiUDP udp;

String ssid = "OM";
String pass = "OMNAIK123";
const int flask_server_port = 5000;
const char *flask_server_ip = "18.218.196.220";

const int kNetworkDelay = 1000;
const int kNetworkTimeout = 30 * 1000;

void wifi_setup()
{
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
}

void lcd_init()
{
	lcd.init();
	lcd.backlight();
	lcd.cursor();
	lcd.clear();
}

void pin_init()
{
	pinMode(BUZZER, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	pinMode(ALC_SENSOR, INPUT);
	pinMode(PULSE_SENSOR, INPUT);

	// Configure button and interrupts
	pinMode(fallCheckButton.PIN, INPUT_PULLUP);
	attachInterrupt(fallCheckButton.PIN, isr, FALLING);

	// Configure buzzer and LED
	ledcSetup(BUZZER_CHANNEL, 1000, 16); // 2 kHz tone
	ledcAttachPin(BUZZER, BUZZER_CHANNEL);

	digitalWrite(RED_LED, LOW);

	// 20 Second Sleep for Alcohol Sensor Warmup
	delay(20000);

	lcd.clear();
}

void pulse_sensor_setup()
{
	pulseSensor.blinkOnPulse(RED_LED);
	pulseSensor.analogInput(PULSE_SENSOR);
	pulseSensor.setThreshold(PULSE_SENSOR_THRESHOLD);

	Serial.println(pulseSensor.begin() ? "Pulse Sensor Connection Successful!" : "Pulse Sensor Connection Failed!");
}

void accelerometer_setup()
{
	fallDetectorMPU.initialize();
	Serial.println(fallDetectorMPU.testConnection() ? "MPU6050 Connection Successful!" : "MPU6050 Connection Failed!");
}

void print_lcd(int bpm, float jerk, float mgL)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("BPM:");
	lcd.print(bpm);
	lcd.setCursor(8, 0);
	lcd.print("BAC:");
	lcd.print(mgL);
	lcd.setCursor(0, 1);
	lcd.print("Jerk:");
	lcd.print(jerk);
}

void print_lcd(String first_line, String second_line)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(first_line);
	lcd.setCursor(0, 1);
	lcd.print(second_line);
}

void playBuzzer(int buzzerFrequency, uint8_t ledState)
{
	ledcWrite(BUZZER_CHANNEL, buzzerFrequency);
	digitalWrite(RED_LED, ledState);
}

void checkFall()
{
	static unsigned long lastTime = 0;

	// Read accelerometer data at specified intervals
	if (millis() - lastTime >= SAMPLE_INTERVAL)
	{
		lastTime = millis();

		int16_t ax, ay, az;
		static float jerkSum = 0;
		float currentJerkMagnitude = 0;

		fallDetectorMPU.getAcceleration(&ax, &ay, &az);

		// Convert accelerometer readings to "g" units
		float accelerationX = ax / 9.8;
		float accelerationY = ay / 9.8;
		float accelerationZ = az / 9.8;

		// Calculate jerk (change in acceleration)
		float jerkX = (accelerationX - prevAccX) / (SAMPLE_INTERVAL / 1000.0);
		float jerkY = (accelerationY - prevAccY) / (SAMPLE_INTERVAL / 1000.0);
		float jerkZ = (accelerationZ - prevAccZ) / (SAMPLE_INTERVAL / 1000.0);

		// Update previous accelerations
		prevAccX = accelerationX;
		prevAccY = accelerationY;
		prevAccZ = accelerationZ;

		// Calculate jerk magnitude
		currentJerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ) / 1000.0;

		// Calculate Rolling Average
		jerkSum -= jerkValues[currentIndex];
		jerkValues[currentIndex] = currentJerkMagnitude;
		jerkSum += currentJerkMagnitude;
		jerkMagnitude = jerkSum / 10;
		currentIndex = (currentIndex + 1) % 10;

		Serial.print(">Jerk: ");
		Serial.print(currentJerkMagnitude);
		Serial.println();

		// Check for fall
		if (currentJerkMagnitude > FALL_THRESHOLD)
		{
			currentJerkMagnitude = 0;
			Serial.println("Fall detected!");

			print_lcd("FALL DETECTED!", "PRESS BUTTON!");

			// Play Buzzer and Set LED to HIGH
			playBuzzer(5000, HIGH);

			delayStartTime = millis();

			// Wait for button press or timeout
			while (millis() - delayStartTime < 5000)
			{
				if (fallCheckButton.pressed)
				{
					fallCheckButton.pressed = false;
					print_lcd("BUTTON PRESSED", "BUZZER STOPPED");
					// Serial.println("Buzzer stopped by button press.");
					currentJerkMagnitude = 0;
					break;
				}
			}
			lcd_init();

			// Stop Buzzer and Set LED to LOW
			playBuzzer(0, LOW);
		}
	}
}

void checkAlcohol()
{
	adcValue = 0;
	float unval = 0;

	for (int i = 0; i < 5; i++)
	{
		unval = analogRead(ALC_SENSOR);
		adcValue += unval;
		delay(10);
	}

	val = (adcValue / 5) * (5.0 / 1024.0);
	mgL = 0.67 * val;

	if (mgL > 3.0)
	{
		Serial.println("BAC Exceeded!");

		print_lcd("BAC EXCEEDED!", "PRESS BUTTON!");

		// Play Buzzer and Set LED to HIGH
		playBuzzer(5000, HIGH);

		delayStartTime = millis();

		// Wait for button press or timeout
		while (millis() - delayStartTime < 5000)
		{
			if (fallCheckButton.pressed)
			{
				fallCheckButton.pressed = false;
				print_lcd("BUTTON PRESSED", "BUZZER STOPPED");
				break;
			}
		}
		lcd_init();

		// Stop Buzzer and Set LED to LOW
		playBuzzer(0, LOW);
	}

	// Send BAC to Serial Plotter
	Serial.print(">BAC: ");
	Serial.print(mgL);
	Serial.println();
}

void checkPulse()
{
	// Read pulse sensor analog signal
	pulse_analog_signal = analogRead(PULSE_SENSOR);

	// Send Pulse to Serial Plotter
	Serial.print(">Signal:");
	Serial.print(pulse_analog_signal);
	Serial.println();

	// Update pulse sensor
	if (pulseSensor.sawStartOfBeat())
	{
		my_bpm = pulseSensor.getBeatsPerMinute();
	}
}

void sendPOSTRequest()
{
	int err = 0;
	WiFiClient c;
	HttpClient http(c);

	String postData = "{\"bpm\": " + String(my_bpm) + ", \"bac\": " + String(mgL) + ", \"jerk\": " + String(jerkMagnitude) + "}";

	http.beginRequest();
	err = http.post(flask_server_ip, flask_server_port, "/api/data", NULL);

	if (err == 0)
	{
		Serial.println("POST request started successfully");

		// Add headers manually after starting the request
		http.sendHeader("Content-Type", "application/json");
		http.sendHeader("Content-Length", postData.length());

		http.print(postData); // Send the JSON data in the body

		err = http.responseStatusCode();
		if (err >= 0)
		{
			Serial.print("Got status code: ");
			Serial.println(err);

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
}

void server_request_task(void *parameter)
{
	while (true)
	{
		sendPOSTRequest();
		vTaskDelay(250 / portTICK_PERIOD_MS); // Run every 5 seconds
	}
}

void setup()
{
	Serial.begin(115200);
	Wire.begin();
	wifi_setup();

	analogReadResolution(10);

	lcd_init();
	print_lcd("TraceBAC CS224P", "Initializing...");

	pin_init();
	delay(1000);
	pulse_sensor_setup();
	accelerometer_setup();

	xTaskCreatePinnedToCore(server_request_task, "HTTP_Request_Task", 10000, NULL, 1, NULL, 0);
}

void loop()
{
	checkFall();
	delay(10);

	checkAlcohol();
	delay(10);

	checkPulse();

	print_lcd(my_bpm, jerkMagnitude, mgL);

	vTaskDelay(250 / portTICK_PERIOD_MS); // Update LCD every 500ms
}