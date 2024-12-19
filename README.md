# ee149FinalProject
## HVAC-Integrated Personal Fan Software

This project intially aimed to create automations strictly in the ESPHome framework to control an fan with a temperature sensor, thermal camera, and rotary encoder. However, defining state and adding more specific automations for unsupported sensors (.yaml) became difficult and as a result the project was transfered into the Arduino IDE. In this the sensors are pulled using timers and WIFI controlled by an MQTT interupt.

Two instances of the state machines for this fan are covered in this repository, one where the automatic state for fan control speed is controlled by the current temperature read by the temperature sensor compared to a desired input temperature. The other updates the fan speed by observing the users temperature throught the thermal camera and attempting to approach a desired temperature.

This specific project uses the following sensors:
- MPC9808 Temperature Sensor
- AMG8833 8x8 Pixel Thermal Camera
- PIR Sensor
- Rotary Encoder
- NeoPixel Ring

This specific project additionally used a Adafruit HUZZAH32 - ESP32 Feather for control. The fan to be controlled can be any 5V USB fan without an internal controller additionally by modifying the circuts, any voltage could by supported with a proper regulator. The important components to beware of would be a properly rated NPN transistor and voltage converter. The fan speed can by altered through a PWM scheme on the NPN transistor connected to the fan, allowing the fan to see an effective voltage up to 5V. Note that the fan will have a lower threshold of PWM that will support actually spinning the fan. In this case the threshold was about 120. The power in this case an input from 0-100 is mapped to 120-255 for actual PWM fan control.

The ESP32 controls the state of the fan which can be in OFF, MANUAL, or AUTOMATIC and will publish the current temperature on the temperature sensor, the max value read by the thermal cmaera, the current fan power on a scale of 0-100, and whether someone is present. The ESP32 does this through MQTT in order to connect using this code, simply configure the ssid and password in the first few lines of the program before uploading as well as the MQTT broker and port information. We plan to add capability to interface this directly with HomeAssistant as a custom project. 

While the .yaml files in this repository are not fully configured, basic automations for ESPHome and the sensors used can be seen implemented in the .yaml files if that method is prefered. 

Here are some basic installation steps conglomerated for ESPHome, 

Installation of ESPHome on an ESP32:
Open desired repository or directory of project.
Optionally create a virtual environment to run and install.
The file requirements.txt contains the necessary dependencies included in this specific instance of the project.
Basic set up:
$ pip install wheel
$ pip install esphome
$ esphome version %%shoud show version if properly installed %%
$ esphome wizard configFile.yaml %%config file can be any desired name
Follow the steps in the setup wizard, our project uses the featheresp32 as the board.
$ esphome dashboard directoryName %%working directory where you want config files to be placed

An example of code added to toggle an LED in the dashboard would be:

output:
  - platform: gpio
    pin: GPIO13
    id: led_gpio

switch:
  - platform: output
    name: "ESP32 LED"
    output: pin13_gpio
    id: led_switch
    restore_mode: ALWAYS_ON
    disabled_by_default: True

