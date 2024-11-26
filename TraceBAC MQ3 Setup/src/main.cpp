#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>

// Download the LiquidCrystal_I2C library from https://github.com/Freenove/Freenove_LCD_Module/blob/main/Freenove_LCD_Module_for_ESP32/C/Libraries/LiquidCrystal_I2C.zip and add the extracted folder to the libdeps folder in .pio folder of the project.

// I2C Connections
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#define GAS_SENSOR 13
#define GREEN_LED 18
#define RED_LED 19
#define BUZZER 13

float adcValue = 0, val = 0, mgL = 0;
bool isDrunk = false;

void print_lcd(float mgl)
{
//   Serial.println(mgL);
  lcd.clear();
  lcd.print(mgL);
}


void setup()
{
    Serial.begin(9600);
    Wire.begin();
    pinMode(GAS_SENSOR, INPUT);
    pinMode(RED_LED, OUTPUT);   
    pinMode(GREEN_LED, OUTPUT); 
    pinMode(BUZZER, OUTPUT);    

    lcd.init();
    lcd.backlight();
    lcd.cursor();
    delay(20000);
    lcd.clear();
}

void loop()
{

    adcValue = 0;
    for (int i = 0; i < 10; i++)
    {
        adcValue += analogRead(GAS_SENSOR);
        delay(10);
    }
    Serial.println(adcValue);
    val = (adcValue / 10) * (5.0 / 1024.0);
    mgL = 0.67 * val;

    // TODO: Instead of taking the immediate value, take the average of the previous 10 readings and put it in the mgL variable.
    if (mgL > 0.8)
    {
        isDrunk = true;
        digitalWrite(BUZZER, HIGH);
        digitalWrite(GREEN_LED, LOW); // Turn LED off.
        digitalWrite(RED_LED, HIGH);  // Turn LED on.
        delay(300);
    }
    else
    {
        isDrunk = false;
        digitalWrite(GREEN_LED, HIGH); // Turn LED on.
        digitalWrite(RED_LED, LOW);    // Turn LED off.
    }
    print_lcd(mgL);
    Serial.print("BAC: ");
    Serial.print(mgL, 3);
    Serial.println("mg/L");
    digitalWrite(BUZZER, LOW);
    delay(100);
}