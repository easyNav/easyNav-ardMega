#include "sprot.h"
#include "sprotint.h"
#include "sprotapi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <HardwareSerial.h>
#include <string.h>

volatile byte_t RECEIVE_BUFFER[PACKET_SIZE] = { 0 };
volatile byte_t SEND_BUFFER[PACKET_SIZE] = { 0 };

// Semaphore for maintaining mutual exclusion of serial port
SemaphoreHandle_t SEMA_SERIAL_PORT;

// Release the serial port semaphore for other tasks
#define giveSema() \
{\
	xSemaphoreGive(SEMA_SERIAL_PORT);\
}

// Block on the serial port semaphore
#define takeSema()\
{\
	xSemaphoreTake(SEMA_SERIAL_PORT, portMAX_DELAY);\
}

void SPROTInit(long baudrate)
{   
	SEMA_SERIAL_PORT = xSemaphoreCreateBinary();
	giveSema();
	
	SERIALPORT.setTimeout(SPROT_SEND_TIMEOUT);
	SERIALPORT.begin(baudrate, SERIAL_8N1);
	SERIALPORT.flush();
}

void SPROTClose()
{
	takeSema();
	
	SERIALPORT.flush();
	
	giveSema();
}

int SPROTReceive(byte_t * buffer, int offset, int length, int timeout)
{
	int recv;
	SPROTPacket * packet = (SPROTPacket *)RECEIVE_BUFFER;
	
	// Block on the serial port semaphore
	takeSema();

	recv = receivePacket(packet, timeout);

	// Check is receive operation was successful
	if (recv == SPROT_ERROR)
	{
		// Release the semaphore
		giveSema();

		return SPROT_ERROR;
	}	
	else
	{
		// Ensure caller's buffer is not overloaded
		recv = (packet->DataLength <= length) ? (recv) : (length);

		// Copy data to caller's buffer
		memcpy((buffer + offset), &(packet->Data), recv);

		// Release the semaphore
		giveSema();

		return recv;
	}
}

void SPROTSend(byte_t * buffer, int offset, int length, int timeout)
{
	SPROTPacket * packet = (SPROTPacket *)SEND_BUFFER;
	
	// Block on the serial port semaphore
	takeSema();

	// Create Serial Protocol data packet for transmission
	createPacket((buffer + offset), length, packet);

	// Send the data packet
	sendPacket(packet);

	giveSema();
}
