#include <Arduino.h> // Ensure ESP32 functions are recognized
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define CLK_PIN 27    // Rotary encoder clock pin
#define DT_PIN 33     // Rotary encoder data pin
#define SW_PIN 15     // Rotary encoder switch pin
#define FAN_PWM_PIN 21 // PWM output pin for fan
#define LED_PIN 14
#define LED_COUNT 24

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// State variables
int currentLED = 0;
bool encoderPrevState = false;
bool buttonPrevState = HIGH; // Assume button is not pressed initially
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms debounce delay

// Modes
enum Mode { OFF, MANUAL, AUTOMATIC };
Mode currentMode = OFF;
Mode lastMode = OFF; // To detect mode changes

void setup() {
  Serial.begin(115200);

  // Rotary Encoder pins
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);

  // NeoPixel initialization
  strip.begin();
  strip.show();
  strip.setBrightness(10);
}

void loop() {
  handleButtonPress();

  // Trigger blink effect only when the mode changes
  if (currentMode != lastMode) {
    switch (currentMode) {
      case OFF:
        Serial.println("Mode: OFF");  
        blink(strip.Color(255, 0, 0), 2, 200); // Red blink for OFF
        break;
      case MANUAL:
        Serial.println("Mode: MANUAL");
        blink(strip.Color(10, 55, 255), 2, 200); // Blue blink for MANUAL
        break;
      case AUTOMATIC:
        Serial.println("Mode: AUTOMATIC");
        blink(strip.Color(85, 255, 9), 2, 200); // Green blink for AUTOMATIC
        break;
    }
    lastMode = currentMode; // Update last mode
  }

  // Run the behavior for the current mode
  switch (currentMode) {
    case OFF:
      strip.clear();
      strip.show();
      break;

    case MANUAL:
      handleManualMode();
      break;

    case AUTOMATIC:
      fillStrip(strip.Color(85, 255, 9)); // Solid Green
      break;
  }

  delay(10); // Small delay for debounce and smoother behavior
}

void handleButtonPress() {
  bool buttonState = digitalRead(SW_PIN);

  if (buttonState != buttonPrevState) {
    if (millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();

      if (buttonState == LOW) { // Button pressed
        currentMode = static_cast<Mode>((currentMode + 1) % 3);
        currentLED = 0; // Reset LED counter in MANUAL mode
      }
    }
  }

  buttonPrevState = buttonState;
}

void handleManualMode() {
  bool clkState = digitalRead(CLK_PIN);
  if (clkState != encoderPrevState) {
    if (digitalRead(DT_PIN) != clkState) {
      // Clockwise: Turn LEDs on one by one
      if (currentLED < LED_COUNT) {
        strip.setPixelColor(currentLED, strip.Color(10, 55, 255)); // Cool Blue
        currentLED++;
      }
    } else {
      // Counter-clockwise: Turn LEDs off one by one
      if (currentLED > 0) {
        currentLED--;
        strip.setPixelColor(currentLED, 0); // Off
      }
    }
    strip.show();
  }
  encoderPrevState = clkState;
}

void blink(uint32_t color, int count, int delayTime) {
  for (int i = 0; i < count; i++) {
    fillStrip(color);
    delay(delayTime);
    fillStrip(strip.Color(0, 0, 0)); // Turn off
    delay(delayTime);
  }
}

void fillStrip(uint32_t color) {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}
