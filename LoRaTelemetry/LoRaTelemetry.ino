/*
 Name:		LoRaTelemetry.ino
 Author:	Mark Reimer
*/

#include <SPI.h>
#include <LoRa.h>
static_assert(SERIAL_TX_BUFFER_SIZE <= 255, "The serial buffer is larger than the max package length of the LoRa library.");

const unsigned long BaudRate = 9600;
const int CSPin = 7;
const int ResetPin = 6;
const int IRQPin = 1;
byte PackageCount = 0;


void setup() {
	Serial.begin(BaudRate);
	while (!Serial);

	LoRa.setPins(CSPin, ResetPin, IRQPin);

	if (!LoRa.begin(433E6)) {             // initialize radio at 433 MHz.
		digitalWrite(LED_BUILTIN, HIGH);
		delay(100);
		digitalWrite(LED_BUILTIN, LOW);
		delay(100);
	}
	LoRa.setSyncWord(0xF9);
}

int RXAvaliable;
unsigned long RXStartTime;
byte Buffer[SERIAL_TX_BUFFER_SIZE];

void loop() {
	RXAvaliable = Serial.available();
	if (RXAvaliable > 0)
	{
		RXStartTime = millis();
		delay(3);

		//If the buffer is still filling and we haven't exceeded the serial buffer and we haven't spent more than 20ms, continue waiting for more serial data.
		while (RXAvaliable != Serial.available() && RXAvaliable < SERIAL_TX_BUFFER_SIZE && millis() - RXStartTime < 20)
		{
			RXAvaliable = Serial.available();
			delay(3);
		}

		Serial.readBytes(Buffer, RXAvaliable);
		sendTelemetry(Buffer, RXAvaliable);
	}

	onReceive(LoRa.parsePacket());
}

void sendTelemetry(byte* outgoing, byte size) {
	byte canSend = 0;
	byte counter = 0;
	do
	{
		if (counter == 3)
			return;

		canSend = LoRa.beginPacket();
		counter++;
	} while (canSend == 0);

	LoRa.write(size);
	LoRa.write(outgoing, size);
	LoRa.write(PackageCount);
	LoRa.endPacket();
	PackageCount++;
}

void onReceive(int packetSize) {
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
	}

	//Transfer the telemetry data to serial.
	Serial.write(Buffer, dataSize);
	PackageCount = id;
}
