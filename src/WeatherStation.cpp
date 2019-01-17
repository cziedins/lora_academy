/*
Lora Academy week4 - WeatherStation (custom payload format)

Tested on Arduino Uno
Lora device used:
	RAK Wisnode Lora (arduino shield with RAK811)
	
Sensors used:
	Sparkfun Soil Moisture Sensor
	LDR
	Adafruit BME680
*/


#include <Arduino.h>
#include "RAK811.h"
#include <SoftwareSerial.h>
#include <stdio.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define debugTXpin 11
#define debugRXpin 10
#define LDRPin A0
#define MoisturePin A1
#define MoisturePowerPin 9

#define loraSerial Serial

#define SEALEVELPRESSURE_HPA (1013.25)

#define WORK_MODE LoRaWAN
#define JOIN_MODE OTAA
#define SEND_EVERY      60000
#define RETRIES         3
#define RETRY_AFTER     5000
#define APP_PORT        1
#define REQUIRE_ACK     0

Adafruit_BME680 bme; // I2C

typedef struct BMEReading {
	uint16_t temperature;
	uint16_t pressure;
	uint16_t humidity;
	uint16_t gas;
	bool error;
} BMEReading;



SoftwareSerial debugSerial(debugRXpin,debugTXpin);

RAK811 lora(loraSerial, debugSerial);


bool joined = false;

//----function declarations----
byte getMoisture(int pin);
byte getLight(int pin);
BMEReading getBMEReading();
void sendSensorData();
void receiveDownLinkMessage();
//----

static unsigned long last = 0;

void setup() {

	pinMode(LED_BUILTIN, OUTPUT);
	
	pinMode(MoisturePowerPin, OUTPUT);
	digitalWrite(MoisturePowerPin, LOW);

	debugSerial.begin(9600);
	while (! debugSerial);
	
	loraSerial.begin(9600);
	while (! loraSerial);

	debugSerial.println(lora.rk_getVersion());
	debugSerial.println(lora.rk_getBand());

	lora.rk_setWorkingMode(WORK_MODE);
	lora.rk_setConfig("adr", "on");
    lora.rk_setConfig("duty", "on");
	
	
	while(loraSerial.available()){
		loraSerial.read();
	}
	
	//Note: devEUI, appEUI and AppKey have been set via manual AT commands on device
	if (lora.rk_joinLoRaNetwork(JOIN_MODE))
	{
		while(last < 20000){
			
			if(loraSerial.available()){
				
				String r = loraSerial.readStringUntil('\n');
				
				if(r.startsWith("at+recv=3,")){
					debugSerial.println(r);
					
					joined = true;
					break;
				}
				
			}
			
			last = millis();
		}
		
		if(!joined){
			debugSerial.println("could not Join");
			while(1);
		}

	}
		
	
	// Set up oversampling and filter initialization (from Adafruit sample)
	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150); // 320*C for 150 ms
	bme.begin();

	
	last = 0;
}

void loop()
{
	
	if ((last == 0) || (millis() - last > SEND_EVERY)) {
		last = millis();
		sendSensorData();
	}
	
	receiveDownLinkMessage();

}

void sendSensorData(){
	byte moisture = getMoisture(MoisturePin);
	byte light = getLight(LDRPin);
	BMEReading bmeReading = getBMEReading();
	
	if(joined){
		
		unsigned char tries = 0;
		
		//char * payload = (char*)malloc(23);
		
		char payload[21];
		
		sprintf(payload, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
		moisture,
		light,
		highByte(bmeReading.temperature), lowByte(bmeReading.temperature),
		highByte(bmeReading.pressure), lowByte(bmeReading.pressure),
		highByte(bmeReading.humidity), lowByte(bmeReading.humidity),
		highByte(bmeReading.gas), lowByte(bmeReading.gas));
		
		while (true) {

			lora.rk_setRate(5);
			
			if (lora.rk_sendData(REQUIRE_ACK, APP_PORT, payload)) {
				return;
			}
			
			tries++;
			if (tries > RETRIES) break;

			debugSerial.println("Error sending data, retrying...");
			
			delay(RETRY_AFTER);
		}
	}
}

void receiveDownLinkMessage(){
	
	String r = lora.rk_recvData();
	
	
	
	if (r.length()) {

		debugSerial.println("RECEIVE: " + r);
		
		if (r.startsWith("at+recv=0,")) {

			if(r.endsWith("01"))
			{
				digitalWrite(LED_BUILTIN, HIGH);
			}
			else if(r.endsWith("00"))
			{
				digitalWrite(LED_BUILTIN, LOW);
			}
		}
	}
	
}

BMEReading getBMEReading()
{
	BMEReading bmeReading;
	
	if (! bme.performReading()) {
		debugSerial.println("Failed to perform reading :(");
		bmeReading.error = true;
		return bmeReading;
	}
	
	debugSerial.print("Temperature = ");
	debugSerial.print(bme.temperature);
	debugSerial.println(" *C");

	debugSerial.print("Pressure = ");
	debugSerial.print(bme.pressure / 100.0);
	debugSerial.println(" hPa");

	debugSerial.print("Humidity = ");
	debugSerial.print(bme.humidity);
	debugSerial.println(" %");

	debugSerial.print("Gas = ");
	debugSerial.print(bme.gas_resistance / 1000.0);
	debugSerial.println(" KOhms");

	debugSerial.println();

	bmeReading.temperature = bme.temperature * 100;
	//lowest and highest atmospheric pressure ever recorded in my country Belgium: 950 and 1050
	//so -900 and *100 for the decimals makes the payload fit in a unint16
	bmeReading.pressure = ((bme.pressure / 100) - 900) * 100; 
	bmeReading.humidity = bme.humidity * 100;
	bmeReading.gas = (bme.gas_resistance / 1000.0) * 100;

	return bmeReading;
}

byte getMoisture(int pin){


	digitalWrite(MoisturePowerPin, HIGH); //turn sensor on
	delay(10);
	
	byte moisture = map(analogRead(MoisturePin), 0, 1023, 0, 255);
	
	digitalWrite(MoisturePowerPin, LOW);//turn the sensor off
	
	debugSerial.print("MOISTURE = ");
	debugSerial.print(moisture);
	debugSerial.println();
	
	
	return moisture;

}

byte getLight(int pin){

	byte light = map(analogRead(pin), 0, 1023, 0, 255);
	
	debugSerial.print("LIGHT = ");
	debugSerial.print(light);
	debugSerial.println();
	
	return light;
}

