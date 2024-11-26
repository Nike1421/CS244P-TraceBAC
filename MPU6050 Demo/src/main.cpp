#include <Wire.h>
#include <MPU6050.h>
#include "time.h"

#define BUZZER_PIN 14
#define BUZZER_CHANNEL 0
#define BEEP_DURATION 5000 // 5 seconds
#define PIN_LED 2

const int sampleInterval = 10;   // Interval in milliseconds between readings
const float fallThreshold = 300000; // Adjust to suit your needs (in m/s^3)
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {12, 0, false};

MPU6050 mpu;
unsigned long button_time = 0;  
unsigned long last_button_time = 0;
bool buttonInterrupted = false;
unsigned long delayStartTime = 0;

void IRAM_ATTR isr() {
    button_time = millis();
    if (button_time - last_button_time > 250) { // Debounce logic
        button1.numberKeyPresses++;
        button1.pressed = true;
        last_button_time = button_time;
        buttonInterrupted = true; // Indicate button press
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(); // SDA to GPIO 2, SCL to GPIO 15

    // Initialize MPU6050
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Configure button and interrupts
    pinMode(button1.PIN, INPUT_PULLUP);
    attachInterrupt(button1.PIN, isr, FALLING);

    // Configure buzzer and LED
    // ledcSetup(BUZZER_CHANNEL, 2000, 16); // 2 kHz tone
    // ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
}

void loop() {
    static unsigned long lastTime = 0;

    // Read accelerometer data at specified intervals
    if (millis() - lastTime >= sampleInterval) {
        lastTime = millis();

        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);

        // Convert accelerometer readings to "g" units
        float accelerationX = ax / 9.8;
        float accelerationY = ay / 9.8;
        float accelerationZ = az / 9.8;

        // Calculate jerk (change in acceleration)
        float jerkX = (accelerationX - prevAccX) / (sampleInterval / 1000.0);
        float jerkY = (accelerationY - prevAccY) / (sampleInterval / 1000.0);
        float jerkZ = (accelerationZ - prevAccZ) / (sampleInterval / 1000.0);

        // Update previous accelerations
        prevAccX = accelerationX;
        prevAccY = accelerationY;
        prevAccZ = accelerationZ;

        // Calculate jerk magnitude
        float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

        Serial.printf("Jerk: %.2f\n", jerkMagnitude);

        // Check for fall
        if (jerkMagnitude > fallThreshold) {
            Serial.println("Fall detected!");
            // ledcWriteTone(BUZZER_CHANNEL, 5000); // Play tone
            // digitalWrite(PIN_LED, HIGH); // Turn on LED
            delayStartTime = millis();

            // Wait for button press or timeout
            while (millis() - delayStartTime < BEEP_DURATION) {
                if (button1.pressed) {
                    button1.pressed = false;
                    Serial.println("Buzzer stopped by button press.");
                    break;
                }
            }

            // ledcWrite(BUZZER_CHANNEL, 0); // Stop buzzer
            // digitalWrite(PIN_LED, LOW); // Turn off LED
        }
    }

    // Handle button press outside of fall detection
    if (button1.pressed) {
        Serial.printf("Button was pressed %u times\n", button1.numberKeyPresses);
        button1.pressed = false;
    }
    delay(100);
}
