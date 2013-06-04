#include <stdio.h>
#include <string.h>

#include "FifoQ.h"


FifoQ::FifoQ(int objSize, int len, Mode  m){
	mObjSize = objSize;
	mLength = len;
	mMode = m;
  	mHead = mTail = mSize = 0;
	mFifoPtr = (char *)malloc(len * objSize);
	sem_init(&mSemaphore, 1, 1);
}

int FifoQ::Put(void *obj){
  int ret = -1;

  sem_wait(&mSemaphore);

  if(mSize < mLength){
	memcpy(mFifoPtr + mHead*mObjSize, obj, mObjSize);
    ++mHead %= mLength;
    mSize++;
    ret = 1;
  }
  else{
    if(mMode == DISCARD){
      ret = -1;
    }
    else{
	  memcpy(mFifoPtr + mHead*mObjSize, obj, mObjSize);
      ++mHead %= mLength;
      mTail++;
      mTail %= mLength;
      mSize = mLength;
      ret = 1;
    }
  }

  sem_post(&mSemaphore);

  return ret;
}

int FifoQ::Get(void *obj){
  int ret = -1;

  sem_wait(&mSemaphore);

  if(mSize > 0){
	memcpy(obj, mFifoPtr + mTail*mObjSize, mObjSize);
    ++mTail %= mLength;
    mSize--;
    ret = 1;
  }
  else{
    ret = -1;
  }

  sem_post(&mSemaphore);

  return ret;
}

int FifoQ::Read(void *obj, int index){
 	if(mSize > 0){
		if(index > mSize){
			return -1;
		}
		else{
			mIndex = mTail + index;
			mIndex %= mLength;
			obj = &mFifoPtr[mIndex];
			return 1;
		}
	}
	else{
		return -1;
	}
	return 1;
}


