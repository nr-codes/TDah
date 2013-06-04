#include <stdio.h>
#include <hw/inout.h>
#include "macros.h"
#include "IoHardware.h"

#include "Msi_P402.h"
#include "Dio82C55.h"
#include "Ruby_MM_x12.h"
#include "Msi_P41x.h"


// constructor
IoHardware::IoHardware(){
	for(port = 0; port < NUM_DIGITAL_PORTS; port++ ){
		debouncedDigitalIn[port] = 0;
		debouncedDigitalInDiff[port] = 0;
	}
}

// destructor
IoHardware::~IoHardware(){

}

// Calls the initialization functions for all of the hardware drivers
int IoHardware::Init(){
	// Analog Output Board
	// Ruby initialization should be run before DigitalIO_3
	// because of this driver resets the board
	AnalogOutBoard = new Ruby_MM_x12();
	DIE_IF(AnalogOutBoard->Init(0x300));

	// Digital I/O
	// Go here to change the type of each port (Input/Output)
	// Actually, the board used is 104-DIO-48E
  	DigitalIO = new Dio82C55(&DigitalInput[0], &DigitalOutput[0], &DigitalPortConfig[0]);
	DIE_IF(DigitalIO->Init(0x330, 0));
	DIE_IF(DigitalIO->PortTypeConfig(Dio82C55::PORTA, Dio82C55::OUTPUT));
	DIE_IF(DigitalIO->PortTypeConfig(Dio82C55::PORTB, Dio82C55::INPUT));
	DIE_IF(DigitalIO->PortTypeConfig(Dio82C55::PORTC, Dio82C55::INPUT));

	DigitalIO_2 = new Dio82C55(&DigitalInput[3], &DigitalOutput[3], &DigitalPortConfig[3]);
	DIE_IF(DigitalIO_2->Init(0x330, 4));
	DIE_IF(DigitalIO_2->PortTypeConfig(Dio82C55::PORTA, Dio82C55::OUTPUT));
	DIE_IF(DigitalIO_2->PortTypeConfig(Dio82C55::PORTB, Dio82C55::INPUT));
	DIE_IF(DigitalIO_2->PortTypeConfig(Dio82C55::PORTC, Dio82C55::INPUT));

	DigitalIO_3 = new Dio82C55(&DigitalInput[6], &DigitalOutput[6], &DigitalPortConfig[6]);
	DIE_IF(DigitalIO_3->Init(0x300, 4));
	DIE_IF(DigitalIO_3->PortTypeConfig(Dio82C55::PORTA, Dio82C55::OUTPUT));
	DIE_IF(DigitalIO_3->PortTypeConfig(Dio82C55::PORTB, Dio82C55::INPUT));
	DIE_IF(DigitalIO_3->PortTypeConfig(Dio82C55::PORTC, Dio82C55::INPUT));

	// Encoder Board
	// Used Encoder Counter Board: MSI-P404-8
	EncoderBoard = new Msi_P402();
	DIE_IF(EncoderBoard->Init(0x400));

	// Analog Input Board #1
	AnalogInBoard = new Msi_P41x(16, &(AnalogInput[0]), &(AnalogChannelConfig[0]), 0);
	DIE_IF(AnalogInBoard->Init(0x240));
	// Confgure channel ranges like this
	//DIE_IF(AnalogInBoard->ConfigChannel( 7, Msi_P41x::UNIPOLAR_10));

	return 1;
}


/**********************************
* All of the following functions are based on ProcessInput and
* ProcessOutput. These two functions read in all inputs and write
* to all outputs, respectively. The other functions read and write
* to the arrays used by these two primary functions.
**********************************/

// This function will read in all the inputs from all the input ports
// and cards and store the results in arrays which other functions access
int IoHardware::ProcessInput(){
	DIE_IF(DigitalIO->ReadAll());
	DIE_IF(DigitalIO_2->ReadAll());
	DIE_IF(DigitalIO_3->ReadAll());

	for(port = 0; port < NUM_DIGITAL_PORTS; port++){
		if(DigitalPortConfig[port] == Dio82C55::INPUT){
			Debounce(port);
		}
	}

	DIE_IF(EncoderBoard->ReadAll());
	
	// This is done for efficiency.  See Msi_P41x.C for explanation
	for(ch = 0; ch < 8; ch++){
		DIE_IF(AnalogInBoard->StartConv(ch));
		DIE_IF(AnalogInBoard->ReadBankChannel(ch));
	}

	AnalogInAvgProcess();

}

// This function calls all output functions for each card,
// this updates the outputs set by other functions
int IoHardware::ProcessOutput(){
	DIE_IF(DigitalIO->WriteAll());
	DIE_IF(DigitalIO_2->WriteAll());
	DIE_IF(DigitalIO_3->WriteAll());

	DIE_IF(AnalogOutBoard->WriteAll());
}

// This is a non-standard Dio82C55 feature particular to AccessIO 104-DIO-48E so it's handled here
void IoHardware::EnableExternalInterrupt(){

	out8(DigitalIO->GetBase() + 0xB, 0x0);

}

void IoHardware::ClearExternalInterrupt(){
	
	out8(DigitalIO->GetBase() + 0xF, 0x0);

}

// This writes the analog value "val" to the array out analog outputs,
// ProcessOutput() is necessary to to actually output this value
void IoHardware::SetAnalogOut(AnalogOutputCh ch, double val){
	if(ch < 0 || ch > DACBOARD_CH)
		FATAL_ERROR("Analog output channel out of range");

	AnalogOutBoard->Output[ch] = val;
}

// This function is used to make sure a digital input port's readings have settled down
void IoHardware::Debounce(int port){
    static unsigned char regA[NUM_DIGITAL_PORTS], regB[NUM_DIGITAL_PORTS], regC[NUM_DIGITAL_PORTS];
	static unsigned activeInputs[NUM_DIGITAL_PORTS];

    // set bits that are changing
    activeInputs[port] = DigitalInput[port] ^ debouncedDigitalIn[port];

    // update 3 bit vertical counters
    regA[port] = regA[port] ^ (regB[port] & regC[port]);
    regB[port] = regB[port] ^ regC[port];
    regC[port] = ~regC[port];

    // reset inactive input counters
    regA[port] &= activeInputs[port];
    regB[port] &= activeInputs[port];
    regC[port] &= activeInputs[port];

    // clear bits except for the ones going though count
    debouncedDigitalIn[port] &= (regA[port] | regB[port] | regC[port]);
    // set bits that are not going through count
    debouncedDigitalIn[port] |= (~(regA[port] | regB[port] | regC[port]) & DigitalInput[port]);

	debouncedDigitalInDiff[port] = debouncedDigitalIn[port] ^ debouncedDigitalInPrev[port];
	debouncedDigitalInPrev[port] = debouncedDigitalIn[port];
}

// This function checks to see if a digital I/O bit is set as an input or output
int IoHardware::IsDigitalInputBit(DigitalInputBit bit){
	if(DigitalPortConfig[bit / 8] == Dio82C55::INPUT){
		return 1;
	}
	else{
		return 0;
	}
}

// This function returns a digital input from the array set by ProcessInput()
int IoHardware::GetDigitalInBit(DigitalInputBit bit){
	if(bit < 0 || bit > NUM_DIGITAL_BITS)
		FATAL_ERROR("Digital input out of range");

	port = bit/8;
	portBit = bit - port * 8;

	if(DigitalPortConfig[port] != Dio82C55::INPUT){
		FATAL_ERROR("Digital bit is not an input");
	}

	return (DigitalInput[port] & (0x1 << portBit)) >> portBit ;
}

// This function returns a digital input from the array set by ProcessInput()
// after calling the debounce function
int IoHardware::GetDebouncedDigitalInBit(DigitalInputBit bit){
	if(bit < 0 || bit > NUM_DIGITAL_BITS)
		FATAL_ERROR("Digital input out of range");

	port = bit/8;
	portBit = bit - port * 8;

	if(DigitalPortConfig[port] != Dio82C55::INPUT){
		FATAL_ERROR("Digital bit is not an input");
	}

	return (debouncedDigitalIn[port] & (0x1 << portBit)) >> portBit ;
}

// This function returns a digital input from the array set by ProcessInput()
// after calling the debounce function and checks for rising/falling edges
int IoHardware::GetDebouncedDigitalInBit(DigitalInputBit bit, edgeType edge){
	if(bit < 0 || bit > NUM_DIGITAL_BITS)
		FATAL_ERROR("Digital input out of range");

	port = bit/8;
	portBit = bit - port * 8;

	if(DigitalPortConfig[port] != Dio82C55::INPUT){
		FATAL_ERROR("Digital bit is not an input");
	}

	if(edge == IoHardware::RISING_EDGE){
		return (((debouncedDigitalIn[port] & debouncedDigitalInDiff[port]) >> portBit) & 0x1);
	}
	else if(edge == IoHardware::FALLING_EDGE){
		return ((((~debouncedDigitalIn[port]) & debouncedDigitalInDiff[port]) >> portBit) & 0x1);
	}
}

// This writes the digital state to the array out digital outputs,
// ProcessOutput() is necessary to to actually output this value
// Only writes the digital state in the array, DigitalOutput[]. 
void IoHardware::SetDigitalOutBit(DigitalOutputBit bit, int state){
	if(bit < 0 || bit > NUM_DIGITAL_BITS)
		FATAL_ERROR("Digital output out of range");

	port = bit/8;
	portBit = bit - port * 8;

	if(DigitalPortConfig[port] != Dio82C55::OUTPUT){
		FATAL_ERROR("Digital bit is not an output");
	}

	if(state){
		DigitalOutput[port] |= 0x1 << portBit;
	}
	else{
		DigitalOutput[port] &= ~(0x1 << portBit);
	}
}

// This function gets the encoder counts from a desired channel read in by
// the ProcessInput() function
int IoHardware::GetEncoderCount(EncoderInputCh ch){
	if(ch < 0 || ch > ENCBOARD_CH)
		FATAL_ERROR("Encoder channel out of range");

	return (EncoderBoard->Input[ch]);
}

// This returns the analog value from the array out analog inputs
// read in by ProcessInput()
double IoHardware::GetAnalogIn(AnalogInputCh ch){
	if(ch < 0 || ch > NUM_ANALOG_CHANNELS)
		FATAL_ERROR("Analog Input channel out of range");

	return AnalogInput[ch];
}

// This increments the average reading and number of readings
void IoHardware::AnalogInAvgProcess(){
	avgVal += AnalogInput[avgCh];
	avgCnt ++;
}

// This initializes the averaging process
void IoHardware::AnalogInAvgStart(AnalogInputCh ch){
	avgVal = 0.;
	avgCnt = 0;
	avgCh = ch;
}

// This calculates the average value from an analog input
double IoHardware::AnalogInAvgGet(){
	return avgVal / ((double)avgCnt);
}


/**********************************
* All of the following functions read in current values
* or output desired values instead of just reading and
* writing to arrays.
**********************************/

// This function is like SetDigitalOutBit but will actually output the value
int IoHardware::WriteDigitalBit(DigitalOutputBit bit, int state){ // DigitalOutputBit -> enum
	int port = bit/8; 
	
	SetDigitalOutBit(bit, state);
	
	switch((port/3)){
		case 0:
			DIE_IF(DigitalIO->WritePort(port%3));
			break;
		case 1:
			DIE_IF(DigitalIO_2->WritePort(port%3));
			break;
		case 2:
			DIE_IF(DigitalIO_3->WritePort(port%3));
			break;
	}
}

// This function is like GetDigitalInBit but will actually read in the current value
int IoHardware::ReadDigitalBit(DigitalInputBit bit){
	int port = bit/8;
	
	switch((port/3)){
		case 0:
			DIE_IF(DigitalIO->ReadPort(port%3));
			break;
		case 1:
			DIE_IF(DigitalIO_2->ReadPort(port%3));
			break;
		case 2:
			DIE_IF(DigitalIO_3->ReadPort(port%3));
			break;
	}
	
	Debounce(port);
	
	return GetDigitalInBit(bit);
}

// This function is like SetAnalogOut but will actually output the value
void IoHardware::WriteAnalogCh(AnalogOutputCh ch, double val){
	SetAnalogOut(ch, val);
	
	AnalogOutBoard->WriteCh(ch);
}

// This function is like GetAnalogIn but will actually read in the current value
double IoHardware::ReadAnalogCh(AnalogInputCh ch){
	DIE_IF(AnalogInBoard->StartConv(ch));
	DIE_IF(AnalogInBoard->ReadBankChannel(ch));

	AnalogInAvgProcess();
	
	return GetAnalogIn(ch);
}

// This function is like GetEncoderCount but will actually read in the current value
// It seems not working.
int IoHardware::ReadEncoder(EncoderInputCh ch){
	if(ch < 0 || ch > ENCBOARD_CH)
		FATAL_ERROR("Encoder channel out of range");
		
	DIE_IF(EncoderBoard->ReadCh(ch));
	//DIE_IF(EncoderBoard->ReadAll());
	
	return (EncoderBoard->Input[ch]);
}

