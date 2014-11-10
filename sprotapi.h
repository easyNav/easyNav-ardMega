// sprotfunc.h
// Function declarations for Serial Protocol API
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Author		: Adnan Hanafi
// Last edited	: 21/9/2014
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef _SPROTAPI_H
#define _SPROTAPI_H

#include "sprot.h"

// Initializes data structures and serial ports. Must be the first function called.
void SPROTInit(long baudrate);

// Receive the specified no. of bytes and store in memory location pointed to by buffer
// Returns the no. of bytes received if successful -1 otherwise
int SPROTReceive(byte_t * buffer, int offset, int length, int timeout);

// Sends the specified no. of bytes located in memory location pointed to by buffer
void SPROTSend(byte_t * buffer, int offset, int length, int timeout);

// Closes an active communications session
void SPROTClose();

#endif
