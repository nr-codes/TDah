#include <stdio.h>
#include <unistd.h>

#include "main.h"
#include "Qnx.h"
#include "userInterface.h"
#include "motor.h"


/********************************************************************
* This file initializes all of the hardware, I/O features,    		*
* and the user interface. For details on the actual functionality	*
* see the individual files. The overall structure of this project	*
* is to have a control loop running which drives the motor and		*
* communicates with the high-speed vision system for object			*
* tracking. 														*
********************************************************************/

// create global pointers so they can be referenced by other functions
IoHardware			*HW;	// has all the hardware IO capabilities
SampleLoopTask		*SampleLoop;	// periodic control loop
MatlabNet			*MNET;	// network communication with MATLAB
ExternalInterrupt 	*ExtInt;	// external interrupt
VisionNet			*VNET;	// network communication with Vision system

double ActualSampleRate;
char LEDtoggle;


int main(){
	LEDtoggle = 0;

	int i;
	sem_t block;

	// Enable Hardware Access - QNX library
	// http://www.qnx.org/developers/docs/6.3.2/neutrino/lib_ref/t/threadctl.html
	ThreadCtl(_NTO_TCTL_IO, NULL);
	
	// Instantiate and Configure Global Modules
	
	// Signal registration is done in individual module's Init()'s
	MNET = new MatlabNet();
	VNET = new VisionNet();
	
	// Hardware management module
	HW = new IoHardware();
	HW->Init();
	
	// Make sure all digital io processing settles down
	for(int i=0; i<20; i++){
		HW->ProcessInput();
	}
	
	// Initialize external interrupt command
	ExtInt = new ExternalInterrupt();
	ExtInt->Init("External Interrupt", 65, 10);
	
	//Enable external interrupt on port C, bit 3 of group 1
	HW->EnableExternalInterrupt();
	HW->ClearExternalInterrupt();
	
	// Instantiate Tasks
	SampleLoop = new SampleLoopTask();
	SampleLoop->Init("SampleLoop", SAMPLE_RATE, &ActualSampleRate, 5);
	
	// Start up MatlabNet after all the signals have been added
	MNET->Init(ActualSampleRate, 14);
	VNET->Init(ActualSampleRate, 55);

	
	// Set up digital output to enable motor
	// Let's make a default as motor off
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
	
	// Start user interface loop - list of commands in table.
	// exits when user hits 'q'
	// See userInterface.C for this function.
	ui();
	
	// Set motor output to zero and disable the amplifier
	SampleLoop->done = 1; // camera command
	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	// first send a zero current command , then motor off. 
	// there is no problem with the other way, but just logically.
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;
		
	delay(10);
}

