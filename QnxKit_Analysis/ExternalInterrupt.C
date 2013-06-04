#include <stdio.h>
#include <math.h>

#include <sys/neutrino.h>
#include <inttypes.h>
#include <sys/syspage.h>

#include "ExternalInterrupt.h"

#include "IoHardware.h"

uint64_t cps, mark, ncycles;
double sec;

ExternalInterrupt::ExternalInterrupt() : InterruptTask(){

}

ExternalInterrupt::~ExternalInterrupt(){

}

int ExternalInterrupt::Init(char *name, int priority, int intNum){

	InterruptTask::Init(name, priority, intNum);
	
	// record cpu cycles per second
	cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
	mark = ClockCycles();

}

void ExternalInterrupt::IntTask(){
	
	ncycles = ClockCycles() - mark;
	mark = ncycles + mark;
	
	sec=(double)ncycles/cps;
	
	printf("External Interrupt (%.3lf sec since last one)\n", sec);
	HW->ClearExternalInterrupt();

}