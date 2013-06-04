#ifndef IoHardware_h
#define IoHardware_h

/********************************************************************
* This class is a high-level enclosure for all the individual		*
* I/O hardware classes. 											*
*																	*
* For this class, there are functions to read and write to digital	*
* and analog ports, read encoder counts, and enum structures where	*
* you can define meaningful pin names. There are also functions		*
* that process all inputs/outputs in bulk and store the readings	*
* in arrays.														*
********************************************************************/

class Dio82C55;
class Msi_P41x;
class Msi_P402;
class Ruby_MM_x12;
class IoHardware;

extern IoHardware *HW;

#define NUM_DIGITAL_PORTS 9 // Number of digial IO ports.  6 ch on the DIO borad. 3 ch on the Analog OUT board.
#define NUM_DIGITAL_BITS (NUM_DIGITAL_PORTS * 8) // Each channel (port) is 8 bits.


class IoHardware{
	public:
		// Constuctor
		IoHardware();
		// Destructor
		~IoHardware();
		
		// Digital Output pin names
		enum DigitalOutputBit{
			// port A of D I/O 0
			DOUT_0 	= 0,
			DOUT_1 	= 1,
			DOUT_2 	= 2,
			DOUT_3 	= 3, 
			DOUT_4 	= 4,
			DOUT_5  = 5,
			DOUT_6  = 6,
			DOUT_7  = 7,
			
			// port A of D I/O 1
			//MOTOR_ENABLE = 24, // enables motor (IN1 of amp)
			//CAMERA_TRIGGER = 25, // triggers camera to take a picture
			
			// port A of D I/O 2
			//DOUT_48 = 48,
			//DOUT_49 = 49 // added by Ji-Chul 
			MOTOR_ENABLE = 48, // enables motor (IN1 of amp)
			CAMERA_TRIGGER = 49 // triggers camera to take a picture
								 // changed by Ji-Chul because the DIO board is not working. 4/1/2013
		};

		// Digital Input pin names
		enum DigitalInputBit{
			// port B of D I/O 0
			DIN_8 	= 8 ,
			DIN_9 	= 9 ,
			DIN_10	= 10,
			DIN_11	= 11,
			DIN_12	= 12,
			DIN_13	= 13,
			DIN_14	= 14,
			DIN_15	= 15,
			
			// port B of D I/O 1
			//FRAME_STATUS = 32, // checks to see if a picture is available
			
			// port C of D I/O 1
			DIN_40	= 40,
			DIN_41	= 41,
			DIN_42	= 42,
			DIN_43	= 43,
			
			// port B of D I/O 2
			FRAME_STATUS = 56, // checks to see if a picture is available
							   // changed by Ji-Chul because the DIO board is not working. 4/1/2013
			// port C of D I/O 2
			DIN_71 = 71
		};

		// Analog Output pin names
		enum AnalogOutputCh{
			AMP_SIGNAL = 0,	// signal to amp
			AOUT_1 = 1,
			AOUT_2 = 2,
			AOUT_3 = 3,
			AOUT_4 = 4,
			AOUT_5 = 5,
			AOUT_6 = 6,
			AOUT_7 = 7
		};

		// Analog Input pin names
		enum AnalogInputCh{
			NUM_ANALOG_CHANNELS = 16,

			AIN_0 = 0,
			ROI = 1, 	// region of interest
			OBJ_X = 2,	// x position of ROI centroid
			OBJ_Y = 3,	// y position of ROI centroid
			AIN_4 = 4,
			AIN_5 = 5,
			AIN_6 = 6,
			AIN_7 = 7
		};

		// Encoder Inputs
		enum EncoderInputCh{
			ENC_0 = 0, // encoder channel being used
			ENC_1 = 1,
			ENC_2 = 2,
			ENC_3 = 3,
			ENC_4 = 4,
			ENC_5 = 5,
			ENC_6 = 6,
			ENC_7 = 7
		};

		int Init();
		int ProcessInput();
		int ProcessOutput();
		
		typedef enum{RISING_EDGE, FALLING_EDGE}edgeType;

		void SetAnalogOut(AnalogOutputCh ch, double val);
		int IsDigitalInputBit(DigitalInputBit bit);
		int GetDigitalInBit(DigitalInputBit bit);
		int GetDebouncedDigitalInBit(DigitalInputBit bit);
		int GetDebouncedDigitalInBit(DigitalInputBit bit, edgeType edge);
		void SetDigitalOutBit(DigitalOutputBit bit, int state);
		int GetEncoderCount(EncoderInputCh ch);
		double GetAnalogIn(AnalogInputCh ch);
		void AnalogInAvgStart(AnalogInputCh ch);
		double AnalogInAvgGet();

		double AnalogInput[NUM_ANALOG_CHANNELS];
		unsigned char AnalogChannelConfig[NUM_ANALOG_CHANNELS];

		unsigned char DigitalInput[NUM_DIGITAL_PORTS]; // the values read from the ports.
		unsigned char DigitalOutput[NUM_DIGITAL_PORTS]; // the values to be pushed to the ports.  
		unsigned char DigitalPortConfig[NUM_DIGITAL_PORTS]; // tells this port is for IN or OUT.
		
		void EnableExternalInterrupt();
		void ClearExternalInterrupt();
		
		// Philip's I/O functions
		int WriteDigitalBit(DigitalOutputBit bit, int state);
		int ReadDigitalBit(DigitalInputBit bit);
		void WriteAnalogCh(AnalogOutputCh ch, double val);
		double ReadAnalogCh(AnalogInputCh ch);
		int ReadEncoder(EncoderInputCh ch);

		int motorStatus;


	private:
		Dio82C55		*DigitalIO;       // Group 1 of Nc_920 board
		Dio82C55		*DigitalIO_2;     // Group 2 of Nc_920 board
		Dio82C55		*DigitalIO_3;     // 82C55 on Diamond Ruby Analog Out Board
		
		Ruby_MM_x12		*AnalogOutBoard;
		Msi_P402 		*EncoderBoard;

		Msi_P41x		*AnalogInBoard;   // P414 (16 ch board with input buffers)

		void Debounce(int port);
		void AnalogInAvgProcess();

		int port;
		int portBit;
		int ch;
		unsigned char debouncedDigitalIn[NUM_DIGITAL_PORTS];
		unsigned char debouncedDigitalInPrev[NUM_DIGITAL_PORTS];
		unsigned char debouncedDigitalInDiff[NUM_DIGITAL_PORTS];
		unsigned int avgCnt;
		double avgVal;
		AnalogInputCh avgCh;
};

#endif // IoHardware_h
