# ESP32-weather-station
ESP32 Weather Station With a BME280 sensor

[![Demo CountPages alpha](https://gifs.com/gif/iot-team17-1-wV4jOm)](https://www.youtube.com/watch?v=ek1j272iAmc)

This repository includes code for two ESP32 devices, a slave and a master, both connected via the ESP-NOW protocol.  

### Master
The master recieves the weather sensory data from the BME280 sensor (temperature, humidity and pressure) and sends it to an Azure cloud server and to the slave.<br>
![Master image](/images/master.png)


### Slave
The slave recieves the the sensory data from the master and displays it on a nokia 5110 screen.<br>
![Slave image](/images/slave.jpg)

