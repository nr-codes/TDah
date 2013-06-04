// Driver for MSI-P402 Encoder interface Board

#ifndef Msi_P402_h
#define Msi_P402_h

#define ENCBOARD_CH	8

class Msi_P402{
    public:
        // Constuctor
        Msi_P402();
        // Destructor
		~Msi_P402();

		  virtual int Init(int base);
		  virtual void Reset();
		  virtual int ReadAll();
		  virtual int ReadCh(int ch);

		  long Input[ENCBOARD_CH];
    private:
		  int mBase;
		  unsigned char mLoByte;
		  unsigned char m2ndByte;
		  unsigned char m3rdByte;
		  unsigned char mHiByte;
		  unsigned char mHiByteCheck;
		  unsigned char mReadCnt;
		  int mCurCh;
		  int mStatus;
		  unsigned char mInitialized;

};

#endif // Msi_P402_h


