#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "main.h"
#include "motor.h"

int startpos = 0;
int setstartpos(int spos);
extern char LEDtoggle;

double QueryReal(char *prompt, double minVal,
                        double maxVal, double defaultVal, char *qi)
{
    double returnVal;

    while (1){
		printf("%s =? [%10.6f] ", prompt, defaultVal);
        gets(qi);
		if (qi[0] == 0x0){
            return(defaultVal);
        }
        else{
            if ( sscanf(qi,"%lf",&returnVal) != 1 ){
                printf("Bad floating point number.\n");
            }
            else if ((returnVal < minVal) || (returnVal > maxVal)){
                printf("Value must be between %f and %f.\n",
                             minVal, maxVal);
            }
            else{
                return(returnVal);
            }
        }
    }
}

int QueryInt(char *prompt, int minVal,
                        int maxVal, int defaultVal, char *qi)
{
    int returnVal;

    while (1){
        printf("%s =? [%ld] ",prompt, defaultVal);
        gets(qi);

        if (qi[0] == 0x0){
            return(defaultVal);
        }
        else{
            if ( sscanf(qi,"%d",&returnVal) != 1 ){
                printf("Bad integer.\n");
            }
            else if ((returnVal < minVal) || (returnVal > maxVal)){
                printf("Value must be between %d and %d.\n",
                             minVal, maxVal);
            }
            else{
                return(returnVal);
            }
        }
    }
}

char QueryChar(char *prompt, char *validvalues, char defaultval, char *buffer)
{
    int n;

    while (1){
        printf("%s =? [%c] ",prompt, defaultval);
        gets(buffer);

		if (buffer[0] == 0x0){
            return(defaultval);
        }
        else{
            for (n = 0; n < strlen(validvalues); n++){
                if (buffer[0] == validvalues[n])
                    return(buffer[0]);
            }
            printf("Value must be one of [%s].\n",validvalues);
        }
    }
}

int QueryString(char *prompt, char *buffer)
{
    int n;

    while (1){
        printf("%s =? ",prompt);
        gets(buffer);

		if (buffer[0] == 0x0){
            return(-1);
        }
        else{
            return(1);
        }
    }
}


char QueryAnyChar(char *prompt, char defaultval, char *qi)
{
    int n;

   printf("%s =? [%c] ",prompt, defaultval);
   gets(qi);

    if (qi[0] == 0x0)
    {
        return(defaultval);
    }
    else
    {
        return(qi[0]);
    }
}

void MainMenu(){
	printf(" a - analog inputs.\n");
	printf(" g - signal gain.\n");
	printf(" o - sample loop overrun.\n");
	printf(" t - timing.\n");
	printf(" f - current [A] input.\n");
	
	
	if(SampleLoop->camera == 0) printf(" c - toggle camera on.\n");
	else printf(" c - toggle camera off.\n");

	if(HW->motorStatus == MOTOR_OFF) printf(" m - enable motor.\n");
	else printf(" m - disable motor.\n");
		
	printf(" e - set start encoder value.\n");
	printf(" s - turn the motor to setted start position\n");
	printf(" l - toggle IR LED detecting\n");

	printf(" q - quit.\n");
	printf(" h - print menu.\n");
	
}

//iVal = QueryInt("Axis (X:1, Y:2, THETA:3)",1,3,1,qi);
//dVal = QueryReal("Value",0.0,10000.0,0,qi);
//cVal = QueryChar("Enable Servo","yn",'n',qi);

void MainProcessCommand(char cmd, char *qi){
	char buf[50];
	int iVal;
	char cVal;
	double dVal;
	int ch;


	switch(cmd){
		case 'h':
			MainMenu();
			break;

		case 'a' :
			ch = QueryInt("Analog Channel",0,16,ch,qi);
			printf("ch %d : %.4lf V\n", ch, HW->ReadAnalogCh((IoHardware::AnalogInputCh) ch));

			//HW->AnalogInAvgStart((IoHardware::AnalogInputCh) ch);
			sleep(1);
			//printf("ch %d : %.4lf V\n", ch, HW->AnalogInAvgGet());
			break;
		
		case 'g':
			SampleLoop->Kp = QueryReal("Value Kp",0.,10000.0,SampleLoop->Kp,qi);
			SampleLoop->Kd = QueryReal("Value Kd",0.,10000.0,SampleLoop->Kd,qi);
			SampleLoop->Ki = QueryReal("Value Ki",0.,10000.0,SampleLoop->Ki,qi);
			break;
		
		case 'o':
			printf("SampleLoop Overrun = %d\n", SampleLoop->mOverRun);
			break;
		
		case 't':
			dVal = QueryReal("Test duration [s]",0.,10000.0,10.,qi);
			SampleLoop->TimingStart(dVal);
			break;
			
		case 'c':
			SampleLoop->camera = !(SampleLoop->camera);
			if(SampleLoop->camera == 0) printf("camera is OFF.\n");
			else printf("camera is ON.\n");
			break;

		case 'm':
			if(HW->motorStatus == MOTOR_OFF){
				HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_ON);
				HW->motorStatus = MOTOR_ON;
				printf("motor is ON.\n");
			}
			else{
				HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
				HW->motorStatus = MOTOR_OFF;
				printf("motor is OFF.\n");
			}
			break;

		case 'f':
			SampleLoop->currentI = QueryReal("Value iCmd",-0.1,0.1,SampleLoop->currentI,qi);
			break;

		case 'e':
			startpos = (HW->GetEncoderCount(IoHardware::ENC_0) % 100000 + 100000) % 100000;
			printf("set start encoder:%d\n",  startpos);
			break;

		case 's':
			if (setstartpos(startpos))
				printf("start position setted!\n");
			else 
				printf("Didn't get it..");
			break;

		case 'l':
			LEDtoggle = !LEDtoggle;
			if (LEDtoggle) SampleLoop->IRLEDdetect();
			break;

		default:
			printf("Bad command %c.  Try 'h'.\n", cmd);
	}

}

void ui(){
	char cmd;
	char inputBuffer[128];
	
	printf("\n");
	printf("********************\n");
	printf("QnxKit				\n");
	printf("Kinea Design, LLC.	\n");
	printf("IRcamera feedback   \n");
	printf("********************\n");
	
	//printf("char:%d, int:%d, float:%d, double:%d.\n", sizeof(char), sizeof(int), sizeof(float), sizeof(double));

	MainMenu();

	while ((cmd = QueryAnyChar("QnxKit", 'h', inputBuffer)) != 'q') {
        MainProcessCommand(cmd, inputBuffer);
	}
}

int setstartpos(int spos){
	char flag = 0;
	int inc = 0, cal, inipos;

	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_ON);
	HW->motorStatus = MOTOR_ON;

	inipos = HW->GetEncoderCount(IoHardware::ENC_0);


	while (!flag){
		cal = abs( (HW->GetEncoderCount(IoHardware::ENC_0) % 100000 + 100000)%100000 - spos);
		if ( cal < 50) flag = 1;
		HW->WriteAnalogCh(IoHardware::AMP_SIGNAL, 0.5);
		//printf("%d, %d, %d\n",HW->GetEncoderCount(IoHardware::ENC_0),HW->GetEncoderCount(IoHardware::ENC_0)%100000, cal);
		//inc++;

		if ( HW->GetEncoderCount(IoHardware::ENC_0) - inipos > 150000) return -1;
	}

	HW->SetAnalogOut(IoHardware::AMP_SIGNAL, 0.0); 
	HW->WriteDigitalBit(IoHardware::MOTOR_ENABLE, MOTOR_OFF);
	HW->motorStatus = MOTOR_OFF;

	return 1;
}