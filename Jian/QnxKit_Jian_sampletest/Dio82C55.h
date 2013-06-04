#ifndef Dio82C55_h
#define Dio82C55_h

//#define Dio82C55_NUM_BITS	48

class Dio82C55{
    public:
        // Constuctor
        Dio82C55(unsigned char *in, unsigned char *out, unsigned char *config);
        // Destructor
        ~Dio82C55();
		
		typedef enum{PORTA=0, PORTB, PORTC} PORT;
		typedef enum{OUTPUT=0, INPUT} IOTYPE;
		
		int Init(int base, int regAoffset);
		int PortTypeConfig(PORT p, IOTYPE t);
		int ReadAll();
		int ReadPort(int port);
		int WriteAll();
		int WritePort(int port);
		
		unsigned char *Input;
		unsigned char *Output;
		unsigned char *PortConfig;
		
		int GetBase(){return mBaseWithoutOffset;}

    private:
		int mBase;
		int mBaseWithoutOffset;
		int mCurPort;
		unsigned int controlRegState;
		
		unsigned char mInitialized;
};

#endif // Dio82C55_h


