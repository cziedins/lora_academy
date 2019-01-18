# Lora Academy Exercices


## Weather station Assignment

- Connect 3 sensors to the ExpLoRer board and read out the sensor data in the Serial Monitor of the Arduino IDE. 
- Add LoRa connectivity to the Arduino Sketch and send the sensor data every 10 minutes during a period of 48 hours. Read the data in the Developer Console. Make sure the ExpLoRer joins the network using OTAA. 
- Change the payload to a human-readable format using the Payload Formats in the developer console
- Setup a Cayenne myDevices dashboard and create graphs to visualize the sensor data over time. Add the public URL of the dashboard in the Discussion section of the course, or post a screenshot.

Recommended sensors to use (can be other):
BMP280 - Barometric Pressure Sensor
DHT 22 - Temperature and humidity sensor
MQ4 - Methane CNG Gas Sensor


### Hardware I used for this assignment

- Arduino Uno
- RAK WisNode Lora
- Sparkfun Soil Moisture sensor
- Adafruit BME680 sensor (temperature, atm pressure, humidity, gas)
- LDR (light intensity)


### Wiring

![Breadboard](./images/weatherStation.jpg) 
![Pinout](./images/weatherStation.png)

### Note
The RAK WisNode Lora board is an arduino shield with a RAK811 module. I didn't stack it on top of the Arduino so I could plug a debugger (the white box on the photo). Just as for the Explorer board the RAK811 aslo accepts AT commands.
However it does require hex string messages instead of a byte array.
The src folder contains the Arduino code.









