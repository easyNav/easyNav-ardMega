// sprot.h
// Constants for Serial Protocol
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Author		: Adnan Hanafi
// Last edited	: 6/10/2014
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef _SPROT_H
#define _SPROT_H

#include <Arduino.h>

// The serial port to use for communications (Serial 0-3 on atmega2560)
// Note for Arduino Mega:
// Serial0 = Serial  --> USB
// Serial1 = Serial1 --> free
// Serial2 = Serial2 --> free
// Serial3 = Serial3 --> free
#define SERIALPORT Serial1

// Protocol constants
#define SPROT_PROTOCOL_MARKER				0xAA							// A start of packet protocol marker
#define PACKET_SIZE							32								// N-bytes fixed packet size
#define HEADER_LENGTH						2								// Header size in bytes
#define FIELD_PM_LENGTH						1								// DT field size in bytes
#define FIELD_DLEN_LENGTH					1								// DATA_LENGTH field size in bytes
#define FIELD_CHECKSUM_LENGTH				1								// CHECKSUM field size in bytes

// The length of the DATA field in bytes
#define FIELD_DATA_LENGTH					(PACKET_SIZE - HEADER_LENGTH - FIELD_CHECKSUM_LENGTH)

#define FIELD_PM_OFFSET						0								// DT field offset in bytes
#define FIELD_DLEN_OFFSET					1								// DATA_LENGTH field offset in bytes
#define FIELD_CHECKSUM_OFFSET				(PACKET_SIZE - 1)				// CHECKSUM field offset in bytes
#define FIELD_DATA_OFFSET					HEADER_LENGTH					// DATA field offset in bytes

// Timeout Parameters
#define SPROT_SEND_TIMEOUT					5000							// Default send timeout in ms
#define SPROT_RECV_TIMEOUT					5000							// Default receive timeout ms

// Serial port default communications speed
#define DEFAULT_SERIAL_BAUDRATE				9600

// The generic error constant
#define SPROT_ERROR							-1

// Define the byte_t type as the smallest unsigned type
typedef unsigned char byte_t;				

#endif

