#ifndef Ruby_MM_x12_h
#define Ruby_MM_x12_h

#define DACBOARD_CH	8

class Ruby_MM_x12{
    public:
        // Constuctor
        Ruby_MM_x12();
        // Destructor
        ~Ruby_MM_x12();

		int Init(int base);
		int WriteAll();
		int WriteCh(int ch);
		void Reset();

		double Output[DACBOARD_CH];

    private:
		int mBase;
		int mCurCh;
		int mCount;
		unsigned int mInitialized;

		void Limit(int);
};

#endif // Ruby_MM_x12_h


