#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

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

#define BUZZER 26
#define RED_LED 25
#define ALC_SENSOR 32
#define PULSE_SENSOR 33
#define BUZZER_CHANNEL 0

#define FALL_THRESHOLD 400
#define SAMPLE_INTERVAL 10
#define BEEP_DURATION 5000
#define BAC_THRESHOLD 3
#define PULSE_SENSOR_THRESHOLD 550

// I2C Connections
MPU6050 fallDetectorMPU(0x69);

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

PulseSensorPlayground pulseSensor;

int pulse_analog_signal = 0;
int my_bpm = 0;
float prevAccX = 0, prevAccY = 0, prevAccZ = 0;
float adcValue = 0, val = 0, mgL = 0;
unsigned long button_time = 0;
unsigned long last_button_time = 0;
bool buttonInterrupted = false;
unsigned long delayStartTime = 0;

String ssid = "OM";
String pass = "OMNAIK123";
const int udpPort = 5000;
const char *serverIP = "3.16.143.108";
WiFiUDP udp; // Create a UDP object

float jerkMagnitude = 0;

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
	{ // Debounce logic
		fallCheckButton.numberKeyPresses++;
		fallCheckButton.pressed = true;
		last_button_time = button_time;
		buttonInterrupted = true; // Indicate button press
	}
}

void wifi_setup()
{
	// WiFi.begin(ssid, pass);

	// while (WiFi.status() != WL_CONNECTED)
	// {
	// 	delay(500);
	// 	Serial.print(".");
	// }

	// Serial.println("");
	// Serial.println("WiFi connected");
	// Serial.println("IP address: ");
	// Serial.println(WiFi.localIP());
	// Serial.println("MAC address: ");
	// Serial.println(WiFi.macAddress());
	// udp.begin(8888); // ESP32 listens on port 8888 for UDP
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

	delay(5000);
	// Replace with 20000
}

void pulse_sensor_setup()
{
	pulseSensor.analogInput(PULSE_SENSOR);
	pulseSensor.blinkOnPulse(RED_LED);
	pulseSensor.setThreshold(PULSE_SENSOR_THRESHOLD);

	if (pulseSensor.begin())
	{
		Serial.println("HeartBeat Sensor found.");
	}
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
		static int sampleCount = 0;

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
		float currentJerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

		// Accumulate jerk magnitude and increment sample count
		jerkSum += currentJerkMagnitude / 1000.0;
		sampleCount++;

		delay(10);
		// Calculate average jerk magnitude every 5 samples
		if (sampleCount == 5)
		{
			jerkMagnitude = jerkSum / 5;
			jerkSum = 0;
			sampleCount = 0;
		}

		Serial.print(">Jerk: ");
		Serial.print(currentJerkMagnitude);
		Serial.println();

		// Check for fall
		if (jerkMagnitude > FALL_THRESHOLD)
		{
			Serial.println("Fall detected!");

			// Play Buzzer and Set LED to HIGH
			playBuzzer(5000, HIGH);

			delayStartTime = millis();

			// Wait for button press or timeout
			while (millis() - delayStartTime < 5000)
			{
				if (fallCheckButton.pressed)
				{
					fallCheckButton.pressed = false;
					// Serial.println("Buzzer stopped by button press.");
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
	float unval = 0;
	adcValue = 0;
	for (int i = 0; i < 5; i++)
	{
		unval = analogRead(ALC_SENSOR);
		adcValue += unval;
		delay(10);
	}
	// adcValue = analogRead(ALC_SENSOR);
	val = (adcValue / 5) * (5.0 / 1024.0);
	mgL = 0.67 * val;

	if (mgL > BAC_THRESHOLD)
	{
		// isDrunk = true;
		// digitalWrite(BUZZER, HIGH);
		// digitalWrite(GREEN_LED, LOW); // Turn LED off.
		// digitalWrite(RED_LED, HIGH);  // Turn LED on.
		// delay(300);
		playBuzzer(5000, HIGH);
	}
	else
	{
		// isDrunk = false;
		// digitalWrite(GREEN_LED, HIGH); // Turn LED on.
		// digitalWrite(RED_LED, LOW);    // Turn LED off.
		// playBuzzer(0, LOW);
	}
	if (fallCheckButton.pressed)
	{
		fallCheckButton.pressed = false;
		playBuzzer(0, LOW);

		// Serial.println("Buzzer stopped by button press.");
	}
	Serial.print("BAC: ");
	// Serial.print(mgL, 3);
	// Serial.println("mg/L");
	// playBuzzer(0, LOW);
}

void checkPulse()
{
	pulse_analog_signal = analogRead(PULSE_SENSOR);
	// Serial.print(">Signal:");
	// Serial.print(pulse_analog_signal); // Send the Signal value to Serial Plotter.
	// Serial.println();

	if (pulseSensor.sawStartOfBeat())
	{
		my_bpm = pulseSensor.getBeatsPerMinute();

		// Serial.print("BPM: ");
		// Serial.println(my_bpm);
	}
}

void fallTask(void *parameter)
{
	while (true)
	{
		checkFall();
		vTaskDelay(10 / portTICK_PERIOD_MS); // Run every 10ms
	}
}

void alcoholTask(void *parameter)
{
	while (true)
	{
		checkAlcohol();
		vTaskDelay(100 / portTICK_PERIOD_MS); // Run every 100ms
	}
}

void pulseTask(void *parameter)
{
	while (true)
	{
		checkPulse();
		vTaskDelay(50 / portTICK_PERIOD_MS); // Run every 50ms
	}
}

void sendUDP()
{
	// Create a message to send
	String message = "{\"bpm\": " + String(my_bpm) + ", \"jerk\": " + String(jerkMagnitude) + ", \"mgL\": " + String(mgL) + "}";

	// Send the UDP packet
	udp.beginPacket(serverIP, udpPort); // Start packet
	udp.print(message);					// Add message
	udp.endPacket();					// Send packet

	Serial.println("UDP packet sent: " + message);

	delay(1000); // Send every 5 seconds
}

void udpTask(void *parameter)
{
	while (true)
	{
		sendUDP();
		vTaskDelay(50 / portTICK_PERIOD_MS); // Run every 5 seconds
	}
}

void setup()
{
	Serial.begin(115200);
	Wire.begin();
	
	
	// wifi_setup();
	
	analogReadResolution(10);

	lcd_init();
	pin_init();

	delay(1000);

	pulse_sensor_setup();

	// Initialize MPU6050
	fallDetectorMPU.initialize();
	// Serial.println(fallDetectorMPU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// Create FreeRTOS tasks
	xTaskCreate(fallTask, "Fall Detection Task", 2048, NULL, 1, NULL);
	xTaskCreate(alcoholTask, "Alcohol Detection Task", 2048, NULL, 1, NULL);
	xTaskCreate(pulseTask, "Pulse Detection Task", 2048, NULL, 1, NULL);
	// xTaskCreate(udpTask, "UDP Send Task", 4096, NULL, 1, NULL);
}

void loop()
{
	print_lcd(my_bpm, jerkMagnitude, mgL);
	vTaskDelay(500 / portTICK_PERIOD_MS); // Update LCD every 500ms
}