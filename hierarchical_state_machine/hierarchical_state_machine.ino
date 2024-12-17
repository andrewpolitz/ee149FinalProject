#include <Arduino.h>
#include <Wire.h>
#include <Ticker.h>
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

#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "NETGEAR_CBE"; // Enter your Wi-Fi name
const char *password = "happybug682";  // Enter Wi-Fi password

// MQTT Broker
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "IoTFan/esp32";
const char *mqtt_username = "IoTFan";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// Variables
int power = 0; // Range 0-100
int curr_temp; // Current temperature
int pref_temp = 24; // Preferred temperature
int camera_max; // max camera pixel temp

bool present = false;
bool sw_press = false;
bool prev_press = false;
bool e_tick = false;
bool e_dir = false;
bool camera_flag = false;
bool camera_active = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms debounce delay

// Create Temperature Sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// AMG88xx IR Sensor
Adafruit_AMG88xx camera;
float pixels[AMG_COLS * AMG_ROWS]; // Sensor 1D data array

// Declare our NeoPixel strip object:
`
Ticker presentTimer;
Ticker tempTimer;
Ticker cameraTimer;
Ticker mqttTimer;

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

 // initialize thermal camera
   if (!camera.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor. Check wiring!");
    while (1) delay(1);
  }
  
  Serial.println("AMG8833 Thermal Camera initialized!");

  
  // start the temp sensor
  tempsensor.setResolution(3); // Set resolution: 0 - low (0.5°C), 1 - medium (0.25°C), 2 - high (0.125°C), 3 - max (0.0625°C)
  tempsensor.wake(); // start temp sensor
  
  delay(250); // let temp sensor stabilize

  updateCurrTemp(); // ensures an initial temperature value
  updateCameraMax(); // grabs an initial picture of the environment

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif

  // configure wifi
  configureMQTT();

  // set up polling timers
  presentTimer.attach(5.0, checkPresent); // checks if someone is present every 5 seconds
  tempTimer.attach(5.0, updateCurrTemp); // updates the temperature every 5 seconds
  mqttTimer.attach(10.0, publishTempState); // sets timer to publish mqtt data
  
}

void loop() {

  client.loop();

  updateMainState();

  // State handling
  switch (mainState) {
    case OFF:
      power = 0;
      camera_flag = false;
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
        Serial.println("Changing to State:" + mainState);
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
    String output = "curr_temp" + String(curr_temp);
    client.publish(topic, output.c_str());
}

void checkPresent() {

  Serial.println("Checked Presence");
  bool pir_present = digitalRead(PIR_PIN) == HIGH;

  if(pir_present){
    camera_flag = true; // flag camera
    Serial.println("Motion Detected");
    cameraTimer.attach(2.0, updateCameraMax);
  } 

  if(!camera_flag) {
     cameraTimer.detach();
  }

  bool camera_present = (camera_max > curr_temp + 2);

  // if the pir detects presence (true) or if camera is flagged and detects presence
  present = pir_present || (camera_present && camera_flag);

  // Reset camera_flag if no presence is detected
  camera_flag = present ? camera_flag : false;
  
}

void publishTempState() {

  String output = "curr_temp: " + String(curr_temp) + "\n camera_max: " +  String(camera_max);
  client.publish(topic, output.c_str());

}

void configureMQTT() {
    //WIFI + MQTT Set Up
  WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }
    Serial.println("Connected to the Wi-Fi network");
    //connecting to a mqtt broker
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // Publish and subscribe
    client.publish(topic, "Hi, I'm your ESP32 smart fan ^^");
    client.subscribe(topic);
}

void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message:");
    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
    Serial.println("-----------------------");
}