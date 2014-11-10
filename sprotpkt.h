// sprothdr.h
// Defines the SPROTPacket type for Serial Protocol
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Author		: Adnan Hanafi
// Last edited	: 6/10/2014
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef _SPROTPKT_H
#define _SPROTPKT_H

#include "sprot.h"

/*
Serial Protocol packet structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

+------+--------+--------+------------+
|  PM  |  DLEN  |  DATA  |  CHECKSUM  |
+------+--------+--------+------------+

PM = Protocol Marker
DLEN = Data Length

*/

typedef struct
{
	byte_t ProtocolMarker;
	byte_t DataLength;
	byte_t Data[FIELD_DATA_LENGTH];
	byte_t Checksum;
} SPROTPacket;

#endif
