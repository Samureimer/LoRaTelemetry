/*
 Name:		LoRaTelemetry.ino
 Author:	Mark Reimer
 Libs to include:
 https://github.com/sandeepmistry/arduino-LoRa
*/

#define SerialDebug

#include <SPI.h>
#include <LoRa.h>

#ifdef SerialDebug
#include <SoftwareSerial.h>
SoftwareSerial debugSerial(10, 11); // RX, TX
#endif

static_assert(SERIAL_RX_BUFFER_SIZE <= 255, "The serial buffer is larger than the max package length of the LoRa library.");
#define BaudRate 9600
const int WaitTimeForUARTPacket = 1.0 / (BaudRate / 13.0) * 1000.0 + 1;
const int CSPin = 7;
const int ResetPin = 6;
const int IRQPin = 1;
byte PackageCount = 0;

void setup()
{
	Serial.begin(BaudRate);
	while (!Serial);

#ifdef SerialDebug 
	debugSerial.begin(9600);
#endif

	LoRa.setPins(CSPin, ResetPin, IRQPin);

	if (!LoRa.begin(433E6)) // initialize radio at 433 MHz.
	{

#ifdef SerialDebug 
		debugSerial.println("Failed to start LoRa module.");
#endif
		while (true)
		{
			digitalWrite(LED_BUILTIN, HIGH);
			delay(100);
			digitalWrite(LED_BUILTIN, LOW);
			delay(100);
		}
	}
	LoRa.setSyncWord(0xF9);
}

int RXAvaliable;
unsigned long RXStartTime;
byte Buffer[SERIAL_RX_BUFFER_SIZE];

void loop() 
{
	RXAvaliable = Serial.available();
	if (RXAvaliable > 0)
	{
		RXStartTime = millis();
		delay(WaitTimeForUARTPacket);

		//If the buffer is still filling and we haven't exceeded the serial buffer and we haven't spent more than 20ms, continue waiting for more serial data.
		while (RXAvaliable != Serial.available() && RXAvaliable < SERIAL_RX_BUFFER_SIZE - 8 && millis() - RXStartTime < 20)
		{
			RXAvaliable = Serial.available();
#ifdef SerialDebug 
			debugSerial.print("Serial Data available: ");
			debugSerial.println(RXAvaliable);
			debugSerial.println("Waiting for more data.");
#endif
			delay(WaitTimeForUARTPacket);
		}

		Serial.readBytes(Buffer, RXAvaliable);
		sendTelemetry(Buffer, RXAvaliable);
	}

	onReceive(LoRa.parsePacket());
}

void sendTelemetry(byte* outgoing, byte size) 
{
	byte canSend = 0;
	byte counter = 0;
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
#endif
	PackageCount++;
}

void onReceive(int packetSize) 
{
	byte dataSize;
	byte id;
	if (packetSize == 0)
		return;

	dataSize = LoRa.read();
	if (LoRa.available() < dataSize)
	{
		//Discard LoRa buffer as it's an invalid package.
		while (LoRa.available())
			LoRa.read();

		return;
	}

	LoRa.readBytes(Buffer, dataSize);
	id = LoRa.read();

	if (id != PackageCount + 1)
	{
		//We have package loss.
#ifdef SerialDebug 
		debugSerial.print("PackageLoss. Expected ID: ");
		debugSerial.print(PackageCount + 1);
		debugSerial.print(" Got ID: ");
		debugSerial.println(id);
#endif
	}

	//Transfer the telemetry data to serial.
	Serial.write(Buffer, dataSize);
	PackageCount = id++;
}
