#include <Wire.h>
#include <Arduino.h>

#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>

#define BUZZER 26
#define RED_LED 25
#define ALC_SENSOR 15
#define PULSE_SENSOR 4
#define BUZZER_CHANNEL 0

#define FALL_THRESHOLD 300000
#define SAMPLE_INTERVAL 10
#define BEEP_DURATION 5000 // 5 seconds


// I2C Connections
MPU6050 fallDetectorMPU(0x69);

// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

float prevAccX = 0, prevAccY = 0, prevAccZ = 0;

float adcValue = 0, val = 0, mgL = 0;

unsigned long button_time = 0;  
unsigned long last_button_time = 0;
bool buttonInterrupted = false;
unsigned long delayStartTime = 0;

float jerkMagnitude = 0;

struct Button
{
	const uint8_t PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};

Button fallCheckButton = {5, 0, false};

void IRAM_ATTR isr() {
    button_time = millis();
    if (button_time - last_button_time > 250) { // Debounce logic
        fallCheckButton.numberKeyPresses++;
        fallCheckButton.pressed = true;
        last_button_time = button_time;
        buttonInterrupted = true; // Indicate button press
    }
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

    delay(20000);
}

void print_lcd(String text, bool thisBool, float mgL)
{
	// TODO: Add the logic to print the text on the LCD.
	lcd.clear();
	lcd.print(text);
	lcd.print(mgL);
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
		jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

		Serial.printf("Jerk: %.2f\n", jerkMagnitude);

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
					Serial.println("Buzzer stopped by button press.");
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
    val = (adcValue / 5) * (5.0 / 4096.0);
    mgL = 0.67 * val;

    if (mgL > 0.3)
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
		playBuzzer(0, LOW);
    }
    print_lcd("BAC: ", false, mgL);
    Serial.print("BAC: ");
    Serial.print(mgL, 3);
    Serial.println("mg/L");
	playBuzzer(0, LOW);
}

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	lcd_init();
	pin_init();

	// Initialize MPU6050
    fallDetectorMPU.initialize();
    Serial.println(fallDetectorMPU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop()
{
	checkFall();
    delay(100);
	checkAlcohol();
	delay(100);
}