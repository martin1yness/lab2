//#define RUNBYSPEED

#include "menu.h"
#include "main.h"
#include "pd.h"

#include <pololu/orangutan.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

// extern GLOBALS


// local global data structures
char receive_buffer[32];
unsigned char receive_buffer_position = 0;
char send_buffer[32];

// A generic function for whenever you want to print to your serial comm window.
// Provide a string and the length of that string. My serial comm likes "\r\n" at 
// the end of each string (be sure to include in length) for proper linefeed.
void print_usb( char *buffer, int n ) {
	serial_send( USB_COMM, buffer, n );
	wait_for_sending_to_finish();
}	
		
//------------------------------------------------------------------------------------------
// Initialize serial communication through USB and print menu options
// This immediately readies the board for serial comm
void init_menu() {	
	// Set the baud rate to 9600 bits per second.  Each byte takes ten bit
	// times, so you can get at most 960 bytes per second at this speed.
	serial_set_baud_rate(USB_COMM, 9600);

	// Start receiving bytes in the ring buffer.
	serial_receive_ring(USB_COMM, receive_buffer, sizeof(receive_buffer));

	//memcpy_P( send_buffer, PSTR("USB Serial Initialized\r\n"), 24 );
	//snprintf( printBuffer, 24, "USB Serial Initialized\r\n");
	//print_usb( printBuffer, 24 );
	print_usb( "USB Serial Initialized\r\n", 24);

	//memcpy_P( send_buffer, MENU, MENU_LENGTH );
	print_usb( MENU, MENU_LENGTH );
	lcd_goto_xy(0, 0);
}

//------------------------------------------------------------------------------------------
// process_received_byte: Parses a menu command (series of keystrokes) that 
// has been received on USB_COMM and processes it accordingly.
// The menu command is buffered in check_for_new_bytes_received (which calls this function).
void process_received_string(const char* buffer)
{
	// Used to pass to USB_COMM for serial communication
	int length;
	char tempBuffer[64];
	
	// parse and echo back to serial comm window (and optionally the LCD)	
	char op_type;
	char* op_input = malloc(sizeof(char)*32);
	int parsed;
	parsed = sscanf(buffer, "%c %s", &op_type, op_input);
	*(op_input + parsed - 3) = '\0';
	length = sprintf( tempBuffer, "\r\nOp:%c C:%s\r\n", op_type, op_input);
	print_usb( tempBuffer, length );
	
	switch (op_type) {
		case 'O':
		case 'o':
			// set duty 
			OCR2B = atoi(op_input);
			break;
		case 'F':
		case 'f':
			// switch direction manually
			PORTC ^= (1 << 6);
			break;
		case 'L':
		case 'l':
			// Toggle Logging Pr, Pm, and T
			loggingOn ^= 0x1;
			break;
		case 'V':
		case 'v':
			// View values for: Kd, Kp, Vm, Pr, Pm, and T			
			#ifdef RUNBYSPEED
			length = sprintf(tempBuffer, "Kd:%.2f Kp:%.2f Vm:%.2f Pr:%.2f Pm:%.2f T:%d\r\n>", G_gainDerivative, G_gainProportional, G_currentMotorSpeed, G_desiredMotorSpeed, G_currentMotorSpeed, G_torque);
			#else
			length = sprintf(tempBuffer, "Kd:%.2f Kp:%.2f Vm:%.2f Pr:%.2f Pm:%.2f T:%d\r\n>", G_gainDerivative, G_gainProportional, G_currentMotorSpeed, G_desiredMotorPosition, G_currentMotorPosition, G_torque);
			#endif
			print_usb(tempBuffer, length);
			break;
		case 'R':
		case 'r':
			// Set reference position in 'counts'
			G_desiredMotorPosition = atof(op_input);			
			break;
		case 'S':
		case 's':
			// Set reference speed in 'counts/second'			
			G_desiredMotorSpeed = atof(op_input);
			if(G_desiredMotorSpeed > 55) {
				G_desiredMotorSpeed = 55; // max speed
			} else if(G_desiredMotorSpeed < -55) {
				G_desiredMotorSpeed = -55; // max neg speed
			}
			break;
		case 'T':
		case 't':
			// Execute Trajectory
			executeTrajectoryFlag = 0x1;
			break;
		case 'P':
			// Increase proportional gain
			G_gainProportional += atof(op_input);
			break;
		case 'p':
			// Decrease proportional gain
			G_gainProportional -= atof(op_input);
			break;
		case 'D':
			// Increase differential gain
			G_gainDerivative += atof(op_input);
			break;
		case 'd':
			// Decrease differential gain
			G_gainDerivative -= atof(op_input);
			break;
		default:
			print_usb( "Bad Option. Try one of {LVRSTPpDd}\r\n", 37 );
			//print_usb( MENU, MENU_LENGTH);
			return;
	}

	free(op_input);
} //end menu()

volatile char recievedChars[32];
volatile unsigned char recievedCharsIdx = 0;
void process_received_byte(char byte)
{
	switch(byte) {
		case '\r':
			recievedChars[recievedCharsIdx] = '\0';
			print_usb("\r\n", 1);
			process_received_string(recievedChars);
			recievedCharsIdx = 0;
			break;
		default:
			recievedChars[recievedCharsIdx] = byte;
			++recievedCharsIdx;
			print_usb(&byte, 1);
	}
}

//---------------------------------------------------------------------------------------
// If there are received bytes to process, this function loops through the receive_buffer
// accumulating new bytes (keystrokes) in another buffer for processing
void check_for_new_bytes_received()
{
	/* 
	The receive_buffer is a ring buffer. The call to serial_check() (you should call prior to this function) fills the buffer.
	serial_get_received_bytes is an array index that marks where in the buffer the most current received character resides. 
	receive_buffer_position is an array index that marks where in the buffer the most current PROCESSED character resides. 
	Both of these are incremented % (size-of-buffer) to move through the buffer, and once the end is reached, to start back at the beginning.
	This process and data structures are from the Pololu library. See examples/serial2/test.c and src/OrangutanSerials/*.*
	
	A carriage return from your comm window initiates the transfer of your keystrokes.
	All key strokes prior to the carriage return will be processed with a single call to this function (with multiple passes through this loop).
	On the next function call, the carriage return is processes with a single pass through the loop.
	The menuBuffer is used to hold all keystrokes prior to the carriage return. The "received" variable, which indexes menuBuffer, is reset to 0
	after each carriage return.
	*/ 
	
	while(serial_get_received_bytes(USB_COMM) != receive_buffer_position)
	{
		// Process the new byte that has just been received.
		process_received_byte(receive_buffer[receive_buffer_position]);

		// Increment receive_buffer_position, but wrap around when it gets to
		// the end of the buffer.
		if (receive_buffer_position == sizeof(receive_buffer)-1)
			{
			receive_buffer_position = 0;
		}
		else
		{
			receive_buffer_position++;
		}
	}
}
	
//-------------------------------------------------------------------------------------------
// wait_for_sending_to_finish:  Waits for the bytes in the send buffer to
// finish transmitting on USB_COMM.  We must call this before modifying
// send_buffer or trying to send more bytes, because otherwise we could
// corrupt an existing transmission.
void wait_for_sending_to_finish()
{
	while(!serial_send_buffer_empty(USB_COMM))
		serial_check();		// USB_COMM port is always in SERIAL_CHECK mode
}

