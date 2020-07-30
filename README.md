# ESP32-weather-station
ESP32 Weather Station With a BME280 sensor

This repository includes code for two ESP32 devices, a slave and a master.

### Master
The master recieves the weather sensory data from the BME280 sensor (temperature, humidity and pressure) and sends it to an Azure cloud server and to the slave.<br>
![Master](/images/master.png)
Format: ![Alt Text](url)


### Master
The slave recieves the the sensory data from the master and displays it on a nokia 5110 screen.
