## Smart Radiator Thermostat

Project for smart radiator thermostat. Detail project description is on [hackster.io](https://www.hackster.io/mhudo/smart-radiator-thermostat-c57a9c)

The thermostats connects to a WiFi access point and to a MQTT broker. It is meant to be used with Home Assistant as it publishes its device capabilities on MQTT topic that Home Assistant is subscribed to and integrates the necessary MQTT topics for it to communicate with Home Assistant.

The project is based on the Infineon [MQTT client example](https://github.com/Infineon/mtb-example-wifi-mqtt-client).