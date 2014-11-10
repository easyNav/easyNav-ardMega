// sprotfunc.h
// Structure and function declarations for internally used by Serial Protocol
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Author		: Adnan Hanafi
// Last edited	: 21/9/2014
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef _SPROTINT_H
#define _SPROTINT_H

#define DUMMY_VALUE 0xAA
#include "sprotpkt.h"

// Creates a Serial Protocol packet
// dataBuffer		Pointer to a memory location where the data to be sent is located
// dataLength		The no. of bytes to send from dataBuffer
// packet			The memory location where the packet will be stored
void createPacket(byte_t * dataBuffer, int dataLength, SPROTPacket * packet);


// Generates a checksum for the given bytes
// Checksum algorithm is currently using byte-oriented XOR-ing
// dataBuffer		Pointer to a memory location containing data on which the checksum will be computed
// length			How many bytes of data should be computed in the checksum
// Returns the checksum value for the given bytes
byte_t generateChecksum(byte_t * buffer, int length);


// Receives a single Serial Protocol data packet
// recvPacket		Pointer to a SPROTPacket structure used to hold the packet
// timeout			The receive operation will fail if timeout secs has been exceeded
int receivePacket(SPROTPacket * recvPacket, int timeout);


// Sends a single Serial Protocol data packet
// packet			Pointer to a SPROTPacket structure used to hold the packet
void sendPacket(SPROTPacket* recvPacket);

#endif
