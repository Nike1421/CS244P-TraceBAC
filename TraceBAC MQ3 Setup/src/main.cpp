#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Download the LiquidCrystal_I2C library from https://github.com/Freenove/Freenove_LCD_Module/blob/main/Freenove_LCD_Module_for_ESP32/C/Libraries/LiquidCrystal_I2C.zip and add the extracted folder to the libdeps folder in .pio folder of the project.

// I2C Connections
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

#define GAS_SENSOR A0
#define GREEN_LED 8
#define RED_LED 9
#define BUZZER 13

float adcValue = 0, val = 0, mgL = 0;
bool isDrunk = false;

void printLCD(float mgL, bool isDrunk)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" BAC: ");
    lcd.print(mgL, 3);
    lcd.print("mg/L   ");
    lcd.setCursor(0, 1);
    if (isDrunk)
    {
        lcd.print("      Drunk     ");
    }
    else
    {
        lcd.print("     Normal    ");
    }
}

void setup()
{
    pinMode(GAS_SENSOR, INPUT);
    pinMode(RED_LED, OUTPUT);   
    pinMode(GREEN_LED, OUTPUT); 
    pinMode(BUZZER, OUTPUT);    

    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("   Welcome To   ");
    lcd.setCursor(0, 1);
    lcd.print("Alcohol Detector");
    delay(2000);
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
    printLCD(mgL, isDrunk);
    digitalWrite(BUZZER, LOW);
    delay(100);
}