#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_AMG88xx.h>

#define CLK_PIN 27    // Rotary encoder clock pin
#define DT_PIN 33     // Rotary encoder data pin
#define SW_PIN 15     // Rotary encoder switch pin
#define FAN_PWM_PIN 21 // PWM output pin for fan

// Variables
int power = 0; // Range 0-100
int temp = 0; // Current temperature
int pref_temp = 24; // Preferred temperature
bool present = false;
bool sw_press = false;
bool e_tick = false;
bool e_dir = false;

// Create Temperature Sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// AMG88xx IR Sensor
Adafruit_AMG88xx amg;
// Thermal data array
float thermalData[8][8]; // 8x8 grid for AMG88xx sensor

// States
enum MainState { OFF, MANUAL, AUTOMATIC };
enum SubState { SUB_OFF, SUB_ON };

MainState mainState = OFF;
SubState subState = SUB_OFF;

// Function prototypes
void handleOffState();
void handleManualState();
void handleAutomaticState();
void updateDutyCycle();

void setup() {
  Serial.begin(115200);

  // Rotaty Encoder
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);

  // Fan PWN
  pinMode(FAN_PWM_PIN, OUTPUT);

  // Initialize Temperature Sensor
  if (!tempsensor.begin(0x18)) { // Default I2C address is 0x18
    Serial.println("Couldn't find MCP9808 sensor! Check wiring.");
    while (1); // Halt execution
  }
  Serial.println("MCP9808 sensor initialized.");
  // Set the sensor to shutdown mode for lower power consumption if not reading frequently
  tempsensor.setResolution(3); // Set resolution: 0 - low (0.5°C), 1 - medium (0.25°C), 2 - high (0.125°C), 3 - max (0.0625°C)
}

void loop() {
  // Check for button clicks to switch modes
  if (sw_press) {
    if (mainState == OFF) mainState = MANUAL;
    else if (mainState == MANUAL) mainState = AUTOMATIC;
    else mainState = OFF;
    sw_press = false; // Reset button press flag
  }

  // State handling
  switch (mainState) {
    case OFF:
      power = 0;
      updateDutyCycle();
      break;

    case MANUAL:
      handleManualState();
      break;

    case AUTOMATIC:
      handleAutomaticState();
      break;
  }
}

void handleManualState() {
  if (!present) {
    subState = SUB_OFF;
    power = 0;
  } else if (e_tick) {
    if (e_dir && power < 100) power += 10;    // Increase power
    else if (!e_dir && power > 0) power -= 10; // Decrease power
  }

  // Update substate
  subState = (power > 0) ? SUB_ON : SUB_OFF;
  updateDutyCycle();
}

void handleAutomaticState() {
  if (!present) {
    subState = SUB_OFF;
    power = 0;
  } else {
    if (temp < pref_temp) {
      int diff = pref_temp - temp;
      if (diff >= 20) power = 100;        // Max power
      else power = 5 * diff;              // Proportional power
    } else {
      power = 0;                          // Turn off
    }
  }

  // Update substate
  subState = (power > 0) ? SUB_ON : SUB_OFF;
  updateDutyCycle();
}

void updateDutyCycle() {
  // Simulate duty cycle with LED brightness
  analogWrite(LED_BUILTIN, map(power, 0, 100, 0, 255));
}
