#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP9808.h>
#include <Adafruit_AMG88xx.h>
#include <Adafruit_NeoPixel.h>

#define SCL 22 // SCL PIN on our ESP32 
#define SDA 23 // SDA PIN on our ESP32

#define CLK_PIN 27    // Rotary encoder clock pin
#define DT_PIN 33     // Rotary encoder data pin
#define SW_PIN 15     // Rotary encoder switch pin

#define FAN_PWM_PIN 21 // PWM output pin for fan

#define AMG_COLS 8    // IR Camera col
#define AMG_ROWS 8    // IR Camera row

#define PIR_PIN 32    // PIR Sensor Pin

#define NEO_PIN 14    // NeoPixel Pin
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LED_COUNT 12

// Variables
int power = 0; // Range 0-100
int curr_temp; // Current temperature
int pref_temp = 24; // Preferred temperature
int camera_max = 0;

bool present = false;
bool sw_press = false;
bool prev_press = false;
bool e_tick = false;
bool e_dir = false;
bool camera_flag = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms debounce delay

// Create Temperature Sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// AMG88xx IR Sensor
Adafruit_AMG88xx camera;
float pixels[AMG_COLS * AMG_ROWS]; // Sensor 1D data array

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

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
  pinMode(CLK_PIN, INPUT_PULLUP);
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);

  // Fan PWN
  pinMode(FAN_PWM_PIN, OUTPUT);

  // PIR Sensor
  pinMode(PIR_PIN, INPUT);

  // Set up I2C communication pins
  Wire.begin(SDA, SCL);

  // Initialize Temperature Sensor
  if (!tempsensor.begin(0x18)) { // Default I2C address is 0x18
    Serial.println("Couldn't find MCP9808 sensor! Check wiring.");
    while (1); // Halt execution
  }

  Serial.println("MCP9808 sensor initialized.");

   if (!camera.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor. Check wiring!");
    while (1) delay(1);
  }
  
  Serial.println("AMG8833 Thermal Camera initialized!");

  
  tempsensor.setResolution(3); // Set resolution: 0 - low (0.5°C), 1 - medium (0.25°C), 2 - high (0.125°C), 3 - max (0.0625°C)
  tempsensor.wake(); // start temp sensor
  
  delay(250); // let temp sensor stabilize

  updateCurrTemp(); // ensures an initial temperature value

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
}

void loop() {

  // Check for button clicks to switch modes
  // updateSWPress();

  // if (sw_press) {
  //   if (mainState == OFF) mainState = MANUAL;
  //   else if (mainState == MANUAL) mainState = AUTOMATIC;
  //   else mainState = OFF;
  // }
  // sw_press = false;

  updateMainState();

  // State handling
  switch (mainState) {
    case OFF:
      Serial.println("In Off Mode");
      power = 0;
      camera_flag = false;
      updateDutyCycle();
      break;

    case MANUAL:
      Serial.println("In Manual Mode");
      handleManualState();
      break;

    case AUTOMATIC:
      Serial.println("In Automatic Mode");
      handleAutomaticState();
      Serial.println(curr_temp);
      Serial.println(camera_max);
      break;
  }
}


void handleManualState() {
  checkPresent();
  updateCurrTemp();

  if (!present) {
    subState = SUB_OFF;
    power = 0;
  } else if (e_tick) {

    if (e_dir && power < 100) {
      power += 10; // Increase power
    } else if (!e_dir && power > 0) {
      power -= 10; // Decrease power
    }

  } 

  // Update substate
  subState = (power > 0) ? SUB_ON : SUB_OFF;
  updateDutyCycle();
}

void handleAutomaticState() {

  checkPresent();
  updateCurrTemp();

  if (!present) {
    subState = SUB_OFF;
    power = 0;
  } else {
    if (curr_temp < pref_temp) {
      int diff = pref_temp - curr_temp;
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

void updateMainState() {
  bool buttonState = digitalRead(SW_PIN);

  if (buttonState != prev_press) {
    if (millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      
      if (buttonState == LOW) {
        mainState = static_cast<MainState>((mainState + 1) % 3);
      } 
    }
  }
  prev_press = buttonState;
}

void updateDutyCycle() {
  // Simulate duty cycle with LED brightness
  analogWrite(FAN_PWM_PIN, map(power, 0, 100, 0, 255));
}

void updateCameraMax() {

  camera.readPixels(pixels);

  int maxTemp = pixels[0];
  for(int i = 1; i < AMG_COLS * AMG_ROWS; i++) {
    if (pixels[i] > maxTemp) {
      maxTemp = pixels[i];
    }
  }

  camera_max = maxTemp;
}

void updateCurrTemp() {
    // updates the temp sensor
    curr_temp = tempsensor.readTempC();
}

void checkPresent() {
 
  bool pir_present = digitalRead(PIR_PIN) == HIGH;

  if(pir_present){
    camera_flag = true; // flag camera
  } 
  if(camera_flag) {
     updateCameraMax();
  }

  bool camera_present = (camera_max > curr_temp + 2);

  // if the pir detects presence (true) or if camera is flagged and detects presence 
  present = pir_present || (camera_present & camera_flag);
  
  present = pir_present || (camera_present && camera_flag);

  // Reset camera_flag if no presence is detected
  camera_flag = present ? camera_flag : false;
}