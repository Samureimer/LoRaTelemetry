/*
 Name:		LoRaTelemetry.ino
 Author:	Mark Reimer
 Libs to include:
 https://github.com/sandeepmistry/arduino-LoRa
*/

#define SerialDebug
#define OTA_Update

#include <SPI.h>
#include <LoRa.h>

#ifdef SerialDebug
//#include <SoftwareSerial.h>
//SoftwareSerial debugSerial(D0, D1); // RX, TX
HardwareSerial debugSerial = Serial1;
#endif

#ifdef OTA_Update
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "***";
const char* password = "***";
#endif

#ifndef SERIAL_RX_BUFFER_SIZE
#define SERIAL_RX_BUFFER_SIZE 253
#endif

static_assert(SERIAL_RX_BUFFER_SIZE <= 253, "The serial buffer is larger than the max package length of the LoRa library.");
#define BaudRate 115200
const float SerialByteTime = 1.0 / (BaudRate / 13.0) * 1000.0;
const int WaitTimeForUARTPacket = SerialByteTime * 2 + 1;
const int CSPin = D8;
const int ResetPin = -1;
uint8_t PackageCount = 0;

void setup()
{
	Serial.begin(BaudRate);
	while (!Serial);

#ifdef SerialDebug 
	debugSerial.begin(115200);
#endif

#ifdef OTA_Update 
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
#ifdef SerialDebug 
	debugSerial.println("Connecting...");
#endif
	while (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
#ifdef SerialDebug 
		debugSerial.println("Connection Failed! Rebooting...");
#endif
	}

	ArduinoOTA.setHostname("pc");

#ifdef SerialDebug 
	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH) {
			type = "sketch";
		}
		else { // U_SPIFFS
			type = "filesystem";
		}

		debugSerial.println("Start updating " + type);
		});

	ArduinoOTA.onEnd([]() {
		debugSerial.println("\nEnd");
		});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		debugSerial.printf("Progress: %u%%\r", (progress / (total / 100)));
		});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
			debugSerial.println("Auth Failed");
		}
		else if (error == OTA_BEGIN_ERROR) {
			debugSerial.println("Begin Failed");
		}
		else if (error == OTA_CONNECT_ERROR) {
			debugSerial.println("Connect Failed");
		}
		else if (error == OTA_RECEIVE_ERROR) {
			debugSerial.println("Receive Failed");
		}
		else if (error == OTA_END_ERROR) {
			debugSerial.println("End Failed");
		}
		});
#endif

	ArduinoOTA.begin();
#ifdef SerialDebug 
	debugSerial.println("Ready");
	debugSerial.print("IP address: ");
	debugSerial.println(WiFi.localIP());
#endif
#endif

	LoRa.setPins(CSPin, ResetPin);
	if (!LoRa.begin(433E6)) // initialize radio at 433 MHz.
	{

#ifdef SerialDebug 
		debugSerial.println("Failed to start LoRa module.");
#endif
		while (true)
		{

#ifndef SerialDebug 
			digitalWrite(LED_BUILTIN, HIGH);
			delay(100);
			digitalWrite(LED_BUILTIN, LOW);
			delay(100);
#endif

#ifdef SerialDebug
			debugSerial.println();
			debugSerial.println("Failed to start LoRa module.");
			delay(1000);
#endif
		}
	}
	LoRa.setSyncWord(0xF9);
}

int RXAvaliable;
unsigned long RXStartTime;
uint8_t Buffer[SERIAL_RX_BUFFER_SIZE];

void loop()
{
#ifdef OTA_Update 
	ArduinoOTA.handle();
#endif

	RXAvaliable = Serial.available();
	if (RXAvaliable > 0)
	{
		RXStartTime = millis();
		delay(WaitTimeForUARTPacket);

		//If the buffer is still filling and we haven't exceeded the serial buffer and we haven't spent more than 20ms, continue waiting for more serial data.
		while (RXAvaliable != Serial.available() && RXAvaliable < SERIAL_RX_BUFFER_SIZE - 8 && millis() - RXStartTime < 60)
		{
			RXAvaliable = Serial.available();
#ifdef SerialDebug 
			debugSerial.print("Serial Data available: ");
			debugSerial.println(RXAvaliable);
			debugSerial.println("Waiting for more data.");
#endif
			delay(WaitTimeForUARTPacket);
		}

		RXAvaliable = Serial.available();
		Serial.readBytes(Buffer, RXAvaliable);
		sendTelemetry(Buffer, RXAvaliable);
	}

	onReceive(LoRa.parsePacket());
}

void sendTelemetry(uint8_t* outgoing, uint8_t size)
{
	uint8_t canSend = 0;
	uint8_t counter = 0;
	do
	{
		if (counter == 3)
		{
#ifdef SerialDebug 
			debugSerial.println("Couldn't begin LoRa package.");
#endif
			return;
		}

		canSend = LoRa.beginPacket();
		counter++;
	} while (canSend == 0);

	LoRa.write(size);
	LoRa.write(outgoing, size);
	LoRa.write(PackageCount);
	LoRa.endPacket();

#ifdef SerialDebug 
	debugSerial.print("Package sent. ID: ");
	debugSerial.print(PackageCount);
	debugSerial.print(" Size: ");
	debugSerial.println(size);
#endif
	PackageCount++;
}

void onReceive(int packetSize)
{
	uint8_t available;
	uint8_t dataSize;
	uint8_t id;
	if (packetSize == 0)
		return;

#ifdef SerialDebug 
	debugSerial.print("Got packet. Size: ");
	debugSerial.println(packetSize);
#endif

	dataSize = LoRa.read();
	available = LoRa.available();

	if (available < dataSize)
	{
#ifdef SerialDebug 
		debugSerial.print("Discarding LoRa buffer. Expected: ");
		debugSerial.print(dataSize);
		debugSerial.print("Got size: ");
		debugSerial.println(available);
#endif
		//Discard LoRa buffer as it's an invalid package.
		while (LoRa.available())
			LoRa.read();

		return;
	}

	LoRa.readBytes(Buffer, dataSize);
	id = LoRa.read();

	if (id != PackageCount)
	{
		//We have package loss.
#ifdef SerialDebug 
		debugSerial.print("PackageLoss. Expected ID: ");
		debugSerial.print(PackageCount);
		debugSerial.print(" Got ID: ");
		debugSerial.println(id);
#endif
	}

#ifdef SerialDebug
	debugSerial.print("RSSI: ");
	debugSerial.println(LoRa.packetRssi());
#endif
	//Transfer the telemetry data to serial.
	Serial.write(Buffer, dataSize);
	PackageCount = id + 1;
}
