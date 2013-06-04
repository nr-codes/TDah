#ifndef FifoQ_h
#define FifoQ_h

#include <semaphore.h>
#include <stdlib.h>

class FifoQ{
	public:
		// overflow handling
  		enum Mode {OVERWRITE, DISCARD};
  		FifoQ(int objSize, int len, Mode  m);
  		~FifoQ(){ free(mFifoPtr); sem_destroy(&mSemaphore);}
  		int Put(void *);
  		int Get(void *);
		int Read(void *, int);
		int Length(){ return mSize; }
		void Reset(){ mHead = mTail = mSize = 0; }

 	private:
		sem_t mSemaphore;
  		int mHead;
  		int mTail;
  		int mSize;
		int mObjSize;
  		int mLength;
		int mIndex;
  		Mode mMode;
  		char *mFifoPtr;
};

#endif // FifoQ_h


