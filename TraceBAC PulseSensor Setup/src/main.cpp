#include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.
#include <Arduino.h>               // Includes Arduino Library.
#include <LiquidCrystal_I2C.h>

#define USE_ARDUINO_INTERRUPTS true // Set-up low-level interrupts for most acurate BPM math.

#define LED_BUILTIN 25
#define PULSE 32

// Om: This threshold value needs to be fine tuned.
int Threshold = 550; 

int Signal;                       
PulseSensorPlayground pulseSensor;
// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2); 

void lcd_init()
{
	lcd.init();
    lcd.backlight();
    lcd.cursor();
    lcd.clear();
}

void print_lcd(int bpm)
{
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("BPM:");
	lcd.print(bpm);
}

void setup()
{

  Serial.begin(115200); // For Serial Monitor

  analogReadResolution(10); // 0 - 1023
	lcd_init();

  pulseSensor.analogInput(PULSE);
  pulseSensor.blinkOnPulse(LED_BUILTIN); 
  pulseSensor.setThreshold(Threshold);

  if (pulseSensor.begin())
  {
    Serial.println("HeartBeat Sensor found.");
  }
}

void loop()
{

  // Convert 0 - 4095 analogRead reading to 0 - 1023
  // Why? Because the PulseSensor Library expects a 0 - 1023 input. (Maybe)
  Signal = analogRead(PULSE);

  Serial.print(">Signal:");
  Serial.print(Signal); // Send the Signal value to Serial Plotter.
  Serial.println();

  if (pulseSensor.sawStartOfBeat())
  {                                               
    int myBPM = pulseSensor.getBeatsPerMinute();  
                                                  
    Serial.print("BPM: ");                        
    Serial.println(myBPM);      
    print_lcd(myBPM);                  
  }

  delay(20);
}