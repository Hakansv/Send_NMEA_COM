
//
// serial.c / serial.cpp
// A simple serial port writing example
// Written by Ted Burke - last updated 13-2-2013

// Adapted for NMEA simulation by Douwe Fokkema 11-11-2014

// To compile with MinGW:
//
//      gcc -o turning.exe turning.c
//
// 
//
// To run:
//
//      turning.exe

// published http://batchloaf.wordpress.com/2013/02/13/writing-bytes-to-a-serial-port-in-c/
//    

#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

char buffer [100] ;

int main()
	{
	printf("Sending at 19200 Bps Com port 5 \n");
	signed int angle;
	printf ("Enter course (integer degrees) \n");
	while (scanf("%d", &angle) != 1)
		{
		while (getchar() != '\n');
		printf ("Try again: ");
		}

	float course = (float) angle;

	printf ("Enter turnrate per 10 seconds (integer seconds) \n ");
	while (scanf("%d", &angle) != 1)
		{
		while (getchar() != '\n');
		printf ("Try again: ");
		}

	char NMEA [100] = "$GPRMC,123519,A,5326.038,N,00611.000,E,022.4,,230394,,W,*41\r\n";

	// Declare variables and structures
	HANDLE hSerial;
	DCB dcbSerialParams = {0};
	COMMTIMEOUTS timeouts = {0};

	// Open COM5 serial port number
	fprintf(stderr, "Opening serial port...");
	hSerial = CreateFile(
		"COM5", GENERIC_READ|GENERIC_WRITE, 0, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	if (hSerial == INVALID_HANDLE_VALUE)
		{
		fprintf(stderr, "Error\n");
		return 1;
		}
	else fprintf(stderr, "OK\n");


	// Set device parameters (38400 baud, 1 start bit,
	// 1 stop bit, no parity)
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (GetCommState(hSerial, &dcbSerialParams) == 0)
		{
		fprintf(stderr, "Error getting device state\n");
		CloseHandle(hSerial);
		return 1;
		}

	dcbSerialParams.BaudRate = CBR_19200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if(SetCommState(hSerial, &dcbSerialParams) == 0)
		{
		fprintf(stderr, "Error setting device parameters\n");
		CloseHandle(hSerial);
		return 1;
		}

	// Set COM port timeout settings
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if(SetCommTimeouts(hSerial, &timeouts) == 0)
		{
		fprintf(stderr, "Error setting timeouts\n");
		CloseHandle(hSerial);
		return 1;
		}

	// Send specified text (remaining command line arguments)
	DWORD bytes_written, total_bytes_written = 0;
	
	float step = ((float) angle) / 100;
	int count = 0;	
	int cx, i;
	char checksum[2];

	while (2> 1) {

	//  main loop follows

		Sleep (80);  // will send heading 10 * / second	
		printf("course  %f  step  %f \n", course, step);
		cx = snprintf ( buffer, 20, "$APHDM,%05.1f,M*XX\r\n", course );
		int test = testsum (buffer);
		cx = snprintf ( checksum, 4, "%02X", test );
		buffer [15] = checksum [0];
		buffer [16] = checksum [1];
		course += step;
		if (course > 360) course -= 360;
		if (course < 0 )  course += 360;

		if (count == 0) 
			{							// write NMEA position every 10 times
		if(!WriteFile(hSerial, NMEA, 61, &bytes_written, NULL))
			{
			fprintf(stderr, "Error\n");
			CloseHandle(hSerial);
			return 1;
			}  
			}

		fprintf(stderr, "%d bytes written NMEA\n", bytes_written);
		count++;    //  send NMEA position every 10 HDM 's
		if (count == 9) count = 0; 

		if(!WriteFile(hSerial, buffer, 19, &bytes_written, NULL))
			{
			fprintf(stderr, "Error\n");
			CloseHandle(hSerial);
			return 1;
			}  
		fprintf(stderr, "%d bytes written HDM\n", bytes_written);
		
		}   // end of while forever

	// Close serial port
	fprintf(stderr, "Closing serial port...");
	if (CloseHandle(hSerial) == 0)
		{
		fprintf(stderr, "Error\n");
		return 1;
		}
	fprintf(stderr, "OK\n");
	Sleep (100000);
	// exit normally
	return 0;
	}

// Calculates the Checksum for the NMEA string
int testsum() {
	int ij = 0;
	int XOR = 0;
	char c;
	
	// Calculate testsum ignoring any $'s in the string
	
	for ( ij = 0; ij < 80; ij++) 
		{ 
		c = buffer[ij];
		if (c == '*') break;
		if (c != '$') XOR ^= c;
		}
	return XOR;
	}

