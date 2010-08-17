// code taken from NI-DAQmx C sample code
#include <stdio.h>
#include <string.h>
#include <NIDAQmx.h>
#include "ni_pci6713.h"

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

#define TIMEOUT 2
#define VMIN -10
#define VMAX 10
#define V_PER_CM (9.0/30)

/*
21 x 
22 y
23 gnd
57 roi
*/

#define PCI6713_AOUT "Dev3/ao0, Dev3/ao1, Dev3/ao2" // refer to: http://sine.ni.com/ds/app/doc/p/id/ds-155/lang/en

#define ROI0_VOLTS VMIN/2
#define ROI1_VOLTS VMAX/2

static TaskHandle taskHandle = 0;
static char errBuff[2048]={'\0'};

int open_analog(void)
{
	int	error = 0;
	float64 data[] = {VMIN, VMIN, VMIN};

	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk (DAQmxCreateTask("",&taskHandle));
	DAQmxErrChk (DAQmxCreateAOVoltageChan(taskHandle, PCI6713_AOUT, "", 
		VMIN, VMAX, DAQmx_Val_Volts, ""));

	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk (DAQmxStartTask(taskHandle));

	/*********************************************/
	// DAQmx Write Code
	/*********************************************/

	// by default all values are VMIN at startup
	DAQmxErrChk (DAQmxWriteAnalogF64(taskHandle, 1, 1, TIMEOUT, DAQmx_Val_GroupByChannel, data, NULL, NULL));

Error:
	if( DAQmxFailed(error) ) {
		memset(errBuff, '\0', sizeof(errBuff));
		DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
		close_analog();
		printf("DAQmx Error: %s\n",errBuff);
		return error;
	}

	return 0;
}

int close_analog(void)
{
	int error = 0;

	if(taskHandle != 0) {
		error = DAQmxStopTask(taskHandle);
		if(!error) {
			error = DAQmxClearTask(taskHandle);
		}
	}

	if( DAQmxFailed(error) ) {
		memset(errBuff, '\0', sizeof(errBuff));
		DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
		printf("DAQmx Error: %s\n",errBuff);
	}

	return error;
}

int output_analog(double x, double y, int roi)
{
	int			error=0;
	float64     data[3];
	
	data[0] = x * V_PER_CM;
	data[1] = y * V_PER_CM;
	if(roi != ROI_LOST) {
		data[2] = roi ? ROI1_VOLTS : ROI0_VOLTS;
	}
	else {
		data[2] = VMIN;
	}


	/*********************************************/
	// DAQmx Write Code
	/*********************************************/
	DAQmxErrChk (DAQmxWriteAnalogF64(taskHandle, 1, 1, TIMEOUT, 
		DAQmx_Val_GroupByChannel, data, NULL, NULL));

Error:
	if( DAQmxFailed(error) ) {
		memset(errBuff, '\0', sizeof(errBuff));
		DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));
		printf("DAQmx Error: %s\n",errBuff);
	}

	return error;
}
