//
//    DynamixelComm.cc		Class to communicate with Dynamixel servos
//
//    Copyright (C) 2018  Birger Johansson
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    See http://www.ikaros-project.org/ for more information.
//
//
//    Created: March 2016
//

#include "DynamixelComm.h"

// MARK: SyncWrite
// AddDataSyncWrite1 will add data to a syncwrite package similar to the addDataBulkWrite. However, AddDataSyncWrite1 does not check the data that will be sent.
// When calling AddDataSyncWrite1(int ID, int adress, unsigned char * data, int dSize) multiple times int adress and int dSize must not change.
// Ex. AddDataSyncWrite1(servo1, adress, value1, length of value) next call AddDataSyncWrite1(servo2, adress, value2, length of value) etc. adress and length of value must be the same.
int DynamixelComm::AddDataSyncWrite1(int ID, int adress, unsigned char * data, int dSize)
{
	// Grab the first adress and dSize. Could be remvoed
	if (syncWriteBufferLength == -1)
	{
		syncWriteAdress = adress;
		syncWriteBlockSize = dSize;
	}
	
#ifdef LOG_COMM
	printf("DynamixelComm (AddDataSyncWrite1) Adding data to syncwrite ID: %i, adress %i, start adress %i, length %i, total length %i\n", ID, adress, syncWriteAdress, dSize, syncWriteBlockSize);
#endif
	
	if (syncWriteBufferLength + 1 + dSize > 1024)  // 1 = id
		return false;
	
	syncWriteBuffer[++syncWriteBufferLength] = ID;
	memcpy(&syncWriteBuffer[++syncWriteBufferLength], data, dSize);
	syncWriteBufferLength+=dSize-1;
	
	//PrintDataSyncWrite1();
	return 0;
}
void DynamixelComm::PrintDataSyncWrite1()
{
	printf("DynamixelComm (PrintDataSyncWrite1) Adress %i, Blocksize %i\n",syncWriteAdress,syncWriteBlockSize);
	for (int i = 0; i <= syncWriteBufferLength; i++) {
		printf("%3i  \t\tBUFFER: \%#04X\t(%3i)\n", i, syncWriteBuffer[i], syncWriteBuffer[i]);
	}
}

int DynamixelComm::SendSyncWrite1()
{
#ifdef LOG_COMM
	printf("DynamixelComm (SendSyncWrite1) Sending SyncWrite command.");
#endif
	if (syncWriteBufferLength == -1)
		return -1;
	
	// Check if the length of the packages is vailed. The maximum buffer size for the dynamixel is 147 bytes.
	if (syncWriteBufferLength+SYNC_WRITE_HEADER_1 > DYNAMIXEL_MAX_BUFFER)
	{
		printf("DynamixelCommunication: Message size is over 143 bytes. Please reduce the number of servos of data sent to the servos\n");
		syncWriteBufferLength = -1;
		return -1;
	}
	
	unsigned char id = 0XFE; 															// Broadcast
	unsigned char length = static_cast<unsigned char>(syncWriteBufferLength+1+4); 		// Length (L+1) X N + 4   (L: Data Length per RX-64, N: the number of RX-64s)
	unsigned char adress = static_cast<unsigned char>(syncWriteAdress);					// Start address to write Data
	unsigned char paramterLenght = static_cast<unsigned char>(syncWriteBlockSize);		// Length of Data to write
	
	// Message
	unsigned char outbuf[256] = {0XFF, 0XFF, id, length, INST_SYNC_WRITE, adress, paramterLenght};
	// Message data
	memcpy(&outbuf[SYNC_WRITE_HEADER_1], &syncWriteBuffer[0], (syncWriteBufferLength+1) * sizeof(unsigned char));
	
	//PrintFullInstructionPackage1(outbuf);
	Send1(outbuf);
	
	// Reset memory
#ifdef LOG_COMM
	printf("DynamixelComm (SendSyncWrite1) Reset SyncWrite memory\n");
#endif
	syncWriteBufferLength = -1;
	syncWriteBlockSize = 0;
	syncWriteAdress = 0;
	return 0;
}

// MARK: Read memory block
bool DynamixelComm::ReadMemoryRange1(int id, unsigned char * buffer, int from, int to)
{
#ifdef LOG_COMM
	printf("DynamixelComm (ReadMemoryRange1): (id:%i) (%i-%i)\n",id, from, to);
#endif
	
	int bytesToRead = to-from+1;
	unsigned char outbuf[256] = {0XFF,0XFF, static_cast<unsigned char>(id), 4, INST_READ, static_cast<unsigned char>(from), static_cast<unsigned char>(bytesToRead), 0X00};
	Send1(outbuf);
	unsigned char inbuf[256];
	int n = Receive1(inbuf);
	
	// Do all message checking here
	if (n == ERROR_NO_HEADER)
	{
#ifdef LOG_COMM_ERROR
		printf("DynamixelComm (ReadMemoryRange1): Did not get the header (4 bytes). Flushing (id:%i)\n",id);
#endif
		FlushIn();
		missingBytesError++;
		return (false);
	}
	if (n == ERROR_CRC)
	{
#ifdef LOG_COMM_ERROR
		printf("DynamixelComm (ReadMemoryRange1): CRC error. Flushing (id:%i)\n",id);
#endif
		FlushIn();
		crcError++;
		return (false);
	}
	if (n == ERROR_NOT_COMPLETE)
	{
#ifdef LOG_COMM_ERROR
		printf("DynamixelComm (ReadMemoryRange1): Did not get all bytes expected. Flushing (id:%i)\n",id);
#endif
		FlushIn();
		notCompleteError++;
		return (false);
	}
	
	// Extended check of the message
	// Checking first bytes id, model etc to make sure these are the right ones.
	int checkBytesToAdress = 6;
	if (bytesToRead < checkBytesToAdress)
		checkBytesToAdress = bytesToRead;
	
	for (int i = 0; i <checkBytesToAdress; i++)
	{
		if (buffer[i] != inbuf[5+i] && buffer[i] != 0)
		{
#ifdef LOG_COMM_ERROR
			printf("DynamixelComm (ReadMemoryRange1): Extended check byte %i of id %i (%i != %i) does not match\n",i, id, buffer[i], inbuf[i]);
#endif
			extendedError++;
			return (false);
		}
	}
	// Parse servo error byte
	getServoError2(inbuf[ERROR_BYTE_1]);

#ifdef LOG_COMM
	printf("DynamixelComm (ReadMemoryRange1): Fill internal buffer from recived buffer. (id:%i)\n",id);
#endif
	
	memcpy(&buffer[0], &inbuf[5], bytesToRead * sizeof(unsigned char));
	return true;
}
// MARK: Send/Recive
void DynamixelComm::Send1(unsigned char * b)
{
#ifdef LOG_COMM
	printf("DynamixelComm (Send1)\n");
#endif
	b[b[3]+3] = CalculateChecksum(b);
	SendBytes((char *)b, b[3]+4);
}

// Calculate timeout
// Timeout = com->time_per_byte*packet_length+ (LATENCY_TIMER*2) + 2.0 // Numbers from Robotis sdk. This might need to be tuned for different platform. Will be testing with raspberry pi 3.

int DynamixelComm::Receive1(unsigned char * b)
{
#ifdef LOG_COMM
	printf("DynamixelComm (Receive1)\n");
#endif
	// read 4 bytes to get header. This only works if the communication is not out of sync.?
	int c = ReceiveBytes((char *)b, 4, (this->time_per_byte*4) + 8 + serialLatency);
	
	if(c < RECIVE_HEADER_1 - 1)
	{
#ifdef LOG_COMM
		printf("DynamixelComm (Receive1): Did not get header (Timed out. Got %i bytes)\n",c);
#endif
		return ERROR_NO_HEADER;
	}
	
	c += ReceiveBytes((char *)&b[4], b[3],  (this->time_per_byte*b[3]) + 8 + serialLatency);
	if(c < b[3])
	{
#ifdef LOG_COMM
		printf("DynamixelComm (Receive1): Did not get all message (Timed out. Got %i bytes)\n",c);
#endif
		return ERROR_NOT_COMPLETE;
	}
	unsigned char checksum = CalculateChecksum(b);
	if(checksum != b[b[3]+3])
		return ERROR_CRC;
	
	return c;
}

// MARK: MISC
bool DynamixelComm::Ping1(int id)
{
#ifdef LOG_COMM
	printf("DynamixelComm (Ping1) ID %i\n", id);
#endif
	Flush();
	unsigned char outbuf[256] = {0XFF, 0XFF, static_cast<unsigned char>(id), 2, INST_PING, 0X00};
	unsigned char inbuf[256];
	Send1(outbuf);
	int n = Receive1(inbuf);
	if (n < 0)
		return (false);
	else
		return (true);
}

void DynamixelComm::Reset1(int id)
{
	printf("Dynamixel: Trying to reset id: %i\n", id);
	unsigned char outbuf[256] = {0XFF, 0XFF, static_cast<unsigned char>(id), 2, INST_RESET, 0X00};
	Send1(outbuf);
	// Recive status packet but do not do anyting with it.
	unsigned char inbuf[256];
	Receive1(inbuf);
}
// MARK: Print
void DynamixelComm::PrintFullInstructionPackage1(unsigned char * package)
{
	PrintPartInstructionPackage1(package, 1, package[3]-2);
}
void DynamixelComm::PrintPartInstructionPackage1(unsigned char * outbuf, int from, int to)
{
	int datalength = to-from+1;                      // Bytes of parameters
	int totalLengthOfPackage = 3 + outbuf[3] + 1;    // The total length of package (3 bytes before length + lenght + crc byte)
	
	int ix = 0;
	int jx = 0;
	printf("\n== Instruction Packet (Send) (%i) ======\n",totalLengthOfPackage);
	for (ix = 0; ix < totalLengthOfPackage-1; ix++)
	{
		switch (ix) {
			case 0:
				printf("============= Start Bytes =======\n");
				break;
			case 2:
				printf("============= ID ================\n");
				break;
			case 3:
				printf("============= Length ============\n");
				break;
			case 4:
				printf("============= Inst ==============\n");
				break;
			case INSTRUCTION_HEADER_1:
				printf("*********** Parameters (%i) ******\n",datalength);
				printf("============= Data bytes ========\n");
				break;
			default:
				break;
		}
		if (ix < INSTRUCTION_HEADER_1)
			printf("%3i  \t\tBUFFER: \%#04X\t(%3i)\n", ix, outbuf[ix], outbuf[ix]);
		else
			printf("%3i (%2i) \tBUFFER: %#04X \t(%3i)\n", ix, jx++, outbuf[ix],outbuf[ix]);
	}
	printf("============ CRC ================\n");
	printf("%3i  \t\tBUFFER: \%#04X\t(%3i)\n", ix, outbuf[ix], outbuf[ix]);
}
void DynamixelComm::PrintMemory1(unsigned char * m, int from, int to)
{
	printf("\n======= Memory Package  ======\n");
	for(int j=from; j<to; j++)
		printf("%3i \t\tBUFFER: %#04X \t(%3i)\n", j, m[j],m[j]);
}

void DynamixelComm::PrintFullStatusPackage1(unsigned char * package)
{
	PrintPartStatusPackage1(package, 1, package[3]-2);
}
void DynamixelComm::PrintPartStatusPackage1(unsigned char * outbuf, int from, int to)
{
	int datalength = to-from+1;                      // Bytes of parameters
	int totalLengthOfPackage = 3 + outbuf[3] + 1;    // The total length of package (3 bytes before length + lenght + crc byte)
	
	int ix = 0;
	int jx = 0;
	printf("\n====== Status Package (%i) ======\n",totalLengthOfPackage);
	for (ix = 0; ix < totalLengthOfPackage-1; ix++)
	{
		switch (ix) {
			case 0:
				printf("============= Start Bytes =======\n");
				break;
			case 2:
				printf("============= ID ================\n");
				break;
			case 3:
				printf("============= Length ============\n");
				break;
			case 4:
				printf("============= Error =============\n");
				break;
			case RECIVE_HEADER_1:
				printf("*********** Parameters (%i) ******\n",datalength);
				printf("============= Data bytes ========\n");
				break;
			default:
				break;
		}
		if (ix < RECIVE_HEADER_1)
			printf("%3i  \t\tBUFFER: \%#04X\t(%3i)\n", ix, outbuf[ix], outbuf[ix]);
		else
			printf("%3i (%2i) \tBUFFER: %#04X \t(%3i)\n", ix, jx++, outbuf[ix],outbuf[ix]);
	}
	printf("============ CRC ================\n");
	printf("%3i  \t\tBUFFER: \%#04X\t(%3i)\n", ix, outbuf[ix], outbuf[ix]);
}

// MARK: CRC
unsigned char DynamixelComm::CalculateChecksum(unsigned char * b)
{
	unsigned char checksum = 0;
	for(int i=0; i<(b[3]+1); i++)
		checksum += b[i+2];
	checksum = ~checksum;
	
	return checksum;
}

// MARK: Error
void DynamixelComm::getServoError1(unsigned char errorByte)
{
	
	
	// Bit 7 0 -
	// Bit 6 Instruction Error In case of sending an undefined instruction or delivering the         action command without the reg_write command, it is set as 1.
	// Bit 5 Overload Error When the current load cannot be controlled by the set Torque, it is set as 1.
	// Bit 4 Checksum Error When the Checksum of the transmitted Instruction Packet is incorrect, it is set as 1.
	// Bit 3 Range Error When a command is out of the range for use, it is set as 1.
	// Bit 2 Overheating Error When internal temperature of Dynamixel is out of the range of operating temperature set in the Control table, it is set as 1.
	// Bit 1 Angle Limit Error When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit , it is set as 1.
	// Bit 0 Input Voltage Error When the applied voltage is out of the range of operating voltage set in the Control table, it is as 1.
	
	ErrorServoInputVoltage = (errorByte >> 0) & 0x1;
	ErrorServoAngleLimit = (errorByte >> 1) & 0x1;
	ErrorServoOverHeating = (errorByte >> 2) & 0x1;
	ErrorServoRange = (errorByte >> 3) & 0x1;
	ErrorServoChecksum = (errorByte >> 4) & 0x1;
	ErrorServoOverload = (errorByte >> 5) & 0x1;
	ErrorServoIntruction = (errorByte >> 6) & 0x1;
	//ErrorServoInputVoltage = (errorByte >> 7) & 0x1;
}
