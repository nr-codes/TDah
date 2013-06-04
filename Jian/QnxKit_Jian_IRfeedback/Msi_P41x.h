#ifndef Msi_P41x_h
#define Msi_P41x_h

//#define MSI_P412_CH	16

class Msi_P41x{
    public:
        // Constuctor
        Msi_P41x(int nCh, double *in, unsigned char *conf, int chOffset);
        // Destructor
        ~Msi_P41x();

		enum ChannelRange{UNIPOLAR_5 = 0x00, UNIPOLAR_10 = 0x10,
		  						BIPOLAR_5  = 0x08,  BIPOLAR_10 = 0x18};

		int Init(int base);
		int ConfigChannel(int ch, ChannelRange range);
		int StartConv(int bankChannel);
		int ReadBankChannel(int bankChannel);

		unsigned char *ChannelConfig;

		double *Input;

    private:
		int Initialized;
		int Base;
		int numCh;
		int numBanks;
		int curCh;
		int curBank;
		int offset;
		int convCount;
		int rawData;
		double sign;
		int ChannelOffset;
	};

#endif // Msi_P41x_h


