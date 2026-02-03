Here is the code to receive input via a MAX4466 sound sensor and output it to a 10-segment LED display

# Arduino Audio VU Meter with MAX4466 and 10-LED Display

## Overview
This Arduino sketch creates an audio volume meter (VU meter) using a MAX4466 microphone sensor and a 10-segment LED display.

## Hardware Requirements
- Arduino board (Uno, Nano, etc.)
- MAX4466 electret microphone amplifier
- 10 LEDs
- 10x 220Ω resistors (current limiting for LEDs)
- Breadboard and jumper wires

## Circuit Connections

### MAX4466 Microphone
- **VCC** → Arduino 5V (or 3.3V)
- **GND** → Arduino GND
- **OUT** → Arduino A0

### LED Connections (Pins 0-9)
Each LED connects as follows:
- **Anode (+)** → Arduino Digital Pin (0-9)
- **Cathode (-)** → 220Ω Resistor → GND

**Note:** Pins 0 and 1 are used for Serial communication (RX/TX). If you need the Serial Monitor for debugging, consider using pins 2-11 instead.

## Arduino Code
```cpp
const int sampleWindow = 50;  // Sample window width in mS (50 mS = 20Hz)
int const AMP_PIN = A0;       // Preamp output pin connected to A0
unsigned int sample;

// LED pins (using digital pins 0-9)
// NOTE: Pins 0 and 1 are also used for Serial communication
// Consider using pins 2-11 instead if you need Serial Monitor
const int ledPins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
const int numLEDs = 10;

// Adjust these thresholds based on your microphone sensitivity
const int minThreshold = 0;    // Minimum peak-to-peak value
const int maxThreshold = 500;  // Maximum peak-to-peak value for full bar

void setup()
{
  Serial.begin(9600);
  
  // Initialize all LED pins as outputs and turn them ON
  for (int i = 0; i < numLEDs; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], HIGH);
    delay(100);
  }
  delay (1000); //They should all be on for a second before turning off
  // Turn all LEDs OFF
  for (int i = 0; i < numLEDs; i++) {
    digitalWrite(ledPins[i], LOW);
  } 
 
}

void loop()
{
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // Collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(AMP_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  
  // Map the peak-to-peak value to number of LEDs (0-10)
  int numLEDsToLight = map(peakToPeak, minThreshold, maxThreshold, 0, numLEDs);
  numLEDsToLight = constrain(numLEDsToLight, 0, numLEDs);
  
  // Update LED display
  updateLEDs(numLEDsToLight);
  
  // Print to Serial Monitor for debugging
  Serial.print("Peak-to-Peak: ");
  Serial.print(peakToPeak);
  Serial.print(" | LEDs lit: ");
  Serial.println(numLEDsToLight);
}

void updateLEDs(int numToLight) {
  // Light up the appropriate number of LEDs
  for (int i = 0; i < numLEDs; i++) {
    if (i < numToLight) {
      digitalWrite(ledPins[i], HIGH);
    } else {
      digitalWrite(ledPins[i], LOW);
    }
  }
}


```

## Calibration

The `maxThreshold` variable (set to 500 by default) determines the audio level needed to light all 10 LEDs. Adjust this value based on:
- Your microphone gain settings
- Ambient noise levels
- Desired sensitivity

### To calibrate:
1. Upload the code and open the Serial Monitor
2. Play audio at your maximum expected volume
3. Note the "Peak-to-Peak" value displayed
4. Set `maxThreshold` to approximately that value
5. Re-upload and test

## Alternative Pin Configuration

If you need Serial Monitor functionality, use pins 2-11 instead:
```cpp
const int ledPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
```

## How It Works

1. **Audio Sampling:** The code continuously reads analog values from the MAX4466 over a 50ms window
2. **Peak Detection:** It calculates the peak-to-peak amplitude (difference between maximum and minimum values)
3. **Mapping:** The amplitude is mapped to a 0-10 scale
4. **LED Display:** The corresponding number of LEDs light up to show the audio level

## Troubleshooting

- **LEDs not lighting:** Check resistor connections and LED polarity
- **All LEDs always on/off:** Adjust `maxThreshold` value
- **Erratic behavior:** Check microphone connections and gain adjustment on MAX4466
- **Serial Monitor issues:** If using pins 0-1 for LEDs, Serial won't work properly

## License

This code is provided as-is for educational and hobbyist purposes.
