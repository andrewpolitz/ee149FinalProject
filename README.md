# ee149FinalProject
HVAC-Integrated Personal Fan Software

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

