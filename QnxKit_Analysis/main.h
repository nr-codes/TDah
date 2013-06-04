#ifndef main_h
#define main_h

#include "IoHardware.h"
#include "SampleLoopTask.h"
#include "MatlabNet.h"
#include "VisionNet.h"
#include "ExternalInterrupt.h"

#define SAMPLE_RATE		800.0 //550.0 //1000.0 //1000.0		// rate at which sample loop will run at (Hz)

extern SampleLoopTask	*SampleLoop; 

#endif
