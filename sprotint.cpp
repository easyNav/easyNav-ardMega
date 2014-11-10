#include "sprot.h"
#include "sprotpkt.h"
#include "sprotint.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>


void createPacket(byte_t * dataBuffer, int dataLength, SPROTPacket * packet)
{
	packet->ProtocolMarker = SPROT_PROTOCOL_MARKER;
	packet->DataLength = dataLength;

	// Copy data to output buffer
	memcpy(packet->Data, dataBuffer, dataLength);

	// Fill unused data bytes with zeroes
	memset((packet->Data + dataLength), 0, (FIELD_DATA_LENGTH - dataLength));

	// All fields are in place, generate checksum for non-empty byte fields except checksum field
	packet->Checksum = generateChecksum((byte_t *)packet, (HEADER_LENGTH + dataLength));
}



byte_t generateChecksum(byte_t * buffer, int length)
{
	byte_t checksum = 0;
	int i;

	for (i = 0; i < length; i++)
	{
		checksum = (checksum ^ buffer[i]);
	}

	return checksum;
}


int receivePacket(SPROTPacket* recvPacket, int timeout)
{
	byte_t checksum;
	int received;
	
	SERIALPORT.setTimeout(timeout);
	received = SERIALPORT.readBytes((char *)recvPacket, PACKET_SIZE);
        
	// Check if receive was successful
	if(received < PACKET_SIZE)
	{
		return SPROT_ERROR;
	}
        	
	// Check for errors
	checksum = generateChecksum((byte_t *)recvPacket, HEADER_LENGTH + recvPacket->DataLength);

	if (checksum != recvPacket->Checksum)
	{
		return SPROT_ERROR;
	}
	else
	{
		return recvPacket->DataLength;
	}
}


void sendPacket(SPROTPacket* recvPacket)
{
	SERIALPORT.write((byte *)recvPacket, PACKET_SIZE);
}
