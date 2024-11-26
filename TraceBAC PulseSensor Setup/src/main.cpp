#include <PulseSensorPlayground.h> // Includes the PulseSensorPlayground Library.
#include <Arduino.h>               // Includes Arduino Library.

#define USE_ARDUINO_INTERRUPTS true // Set-up low-level interrupts for most acurate BPM math.

#define LED_BUILTIN 32
#define PULSE 34

// Om: This threshold value needs to be fine tuned.
int Threshold = 550; 

int Signal;                       
PulseSensorPlayground pulseSensor; 

void setup()
{

  Serial.begin(115200); // For Serial Monitor

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
  Signal = analogRead(PULSE) / 4;

  Serial.print(">Signal:");
  Serial.print(Signal); // Send the Signal value to Serial Plotter.
  Serial.println();

  if (pulseSensor.sawStartOfBeat())
  {                                               
    int myBPM = pulseSensor.getBeatsPerMinute();  
                                                  
    Serial.print("BPM: ");                        
    Serial.println(myBPM);                        
  }

  delay(20);
}