#include<stdio.h>
#include <hw/inout.h>

#include "Dio82C55.h"

#define PORTA_REG 		0
#define PORTB_REG 		1
#define PORTC_REG		2
#define CONTROL_REG		3

#define A_IN			0x10
#define B_IN			0x02
#define C_IN 			0x09
#define CTRL			0x80

Dio82C55::Dio82C55(unsigned char *in, unsigned char *out, unsigned char *config){
	Input = in;
	Output = out;
	PortConfig = config;

	mInitialized = 0;
}

Dio82C55::~Dio82C55(){

}

int Dio82C55::Init(int base, int regAoffset){
	mBaseWithoutOffset = base;
	mBase = base + regAoffset;
	
	// Configure all ports for input
	out8(mBase + CONTROL_REG, CTRL | A_IN | B_IN | C_IN);
	
	controlRegState = CTRL | A_IN | B_IN | C_IN;
	
	for(mCurPort = 0; mCurPort < 3; mCurPort++){
		Input     [mCurPort] = 0;
		Output    [mCurPort] = 0;
		PortConfig[mCurPort] = Dio82C55::INPUT;
	}
	
	mInitialized = 1;
}

int Dio82C55::PortTypeConfig(PORT p, IOTYPE t){
	unsigned char mask;
	
	// check the selected port
	switch(p){
		case PORTA:
			mask = A_IN;
		break;
		case PORTB:
			mask = B_IN;
		break;
		case PORTC:
			mask = C_IN;
		break;

		default:
			return -1;
	}
	
	// send the port configuration to the digital I/O card
	if(t == INPUT){
		controlRegState |= mask;
		out8(mBase + CONTROL_REG, controlRegState);
	}
	else{
		controlRegState &= (~mask);
		out8(mBase + CONTROL_REG, controlRegState);
	}
	
	// save port configuration to the array
	PortConfig[p] = t;
	
	return 1;
}

int Dio82C55::ReadAll(){
	if(!mInitialized)
		return -1;

	// read in all digital inputs and save them to an array
	for(mCurPort = 0; mCurPort < 3; mCurPort++){
		if(PortConfig[mCurPort] == Dio82C55::INPUT){
			Input[mCurPort] = in8(mBase + mCurPort);
		}
	}

	return 1;
}

int Dio82C55::WriteAll(){
	if(!mInitialized)
		return -1;

	// read values set in the output array and send them to the card
	for(mCurPort = 0; mCurPort < 3; mCurPort++){
		if(PortConfig[mCurPort] == Dio82C55::OUTPUT){
			out8(mBase + mCurPort, Output[mCurPort]);
		}
	}

	return 1;
}

int Dio82C55::WritePort(int port){
	if(!mInitialized)
		return -1;
	
	if(PortConfig[port] != Dio82C55::OUTPUT)
		return -1;
	
	// write output to a single port
	out8(mBase + port, Output[port]);
	
	return 1;
}

int Dio82C55::ReadPort(int port){
	if(!mInitialized)
		return -1;
		
	if(PortConfig[port] != Dio82C55::INPUT)
		return -1;
	
	// read in values from a single port
	Input[port] = in8(mBase + port);
	
	return 1;
}
