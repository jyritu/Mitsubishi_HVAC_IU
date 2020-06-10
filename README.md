# Mitsubishi_HVAC_IU
Mitsubishi HVAC interface unit, 10.06.2020
This is firmware for Expressif WROOM module. With this software you are able to establish MQTT control for mitsubishi kirigamine HVAC unit.
My test setup was running with raspberry pi 3 with mosquitto mqtt broker. Used interface / automation is running also on top of RPi with node-red.

MQTT topics are suited for my purpose, but can be easily changed. Code also contains "heart-beat" function, what makes is possible to detect when UI is running.

Please understand this code is released without warranty but is fully fuctional in my test environment.
