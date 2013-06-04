#include <stdio.h>
#include <hw/inout.h>

#include "Msi_P402.h"

#define BYTE_CONSIST_ATTEMPTS	5

Msi_P402::Msi_P402(){
	mInitialized = 0;

	for(mCurCh = 0; mCurCh < ENCBOARD_CH; mCurCh++){
		Input[mCurCh] = 0;
	}

}

Msi_P402::~Msi_P402(){

}

int Msi_P402::Init(int base){
	mBase = base;

	mInitialized = 1;

	Reset();
}

void Msi_P402::Reset(){
	if(!mInitialized)
		return;

	for(mCurCh = 0; mCurCh < ENCBOARD_CH; mCurCh++){
		out8(mBase + mCurCh, 0xFF);
	}

}


int Msi_P402::ReadAll(){

	if(!mInitialized)
		return -1;

	mStatus = 1;
	for(mCurCh = 0; mCurCh < ENCBOARD_CH; mCurCh++){

		mReadCnt = 0;
		while(mReadCnt < BYTE_CONSIST_ATTEMPTS){ // < 5
			mHiByte  = in8(mBase + (mCurCh << 2) + 3);
			m3rdByte = in8(mBase + (mCurCh << 2) + 2);
			m2ndByte = in8(mBase + (mCurCh << 2) + 1);
			mLoByte  = in8(mBase + (mCurCh << 2)    );

			mHiByteCheck = in8(mBase + (mCurCh << 2) + 3);

			// upper 3 bytes must be read twice and be checked if they are the same, for consistency
			//if(mHiByte == mHiByteCheck){
			if (mHiByte == mHiByteCheck &
				m3rdByte == in8(mBase + (mCurCh << 2) + 2) &
				m2ndByte == in8(mBase + (mCurCh << 2) + 1) ) {
				//mLoByte == in8(mBase + (mCurCh << 2)    ) ) {
				Input[mCurCh] = (mHiByte << 24) | (m3rdByte << 16) | (m2ndByte << 8) | mLoByte;
				break;
			}
			else{
				mReadCnt++;
			}
		}
		if(mReadCnt >= BYTE_CONSIST_ATTEMPTS){
			mStatus = -1;
			return mStatus;
		}
	}

	return mStatus;
}

int Msi_P402::ReadCh(int ch){
	if(!mInitialized)
		return -1;

	mStatus = 1;

	mReadCnt = 0;
	while(mReadCnt < BYTE_CONSIST_ATTEMPTS){
		mHiByte  = in8(mBase + (ch << 2) + 3);
		m3rdByte = in8(mBase + (ch << 2) + 2);
		m2ndByte = in8(mBase + (ch << 2) + 1);
		mLoByte  = in8(mBase + (ch << 2)    );

		mHiByteCheck = in8(mBase + (ch << 2) + 3);

		// upper 3 bytes must be read twice and be checked if they are the same, for consistency
		if (mHiByte == mHiByteCheck &
			m3rdByte == in8(mBase + (mCurCh << 2) + 2) &
			m2ndByte == in8(mBase + (mCurCh << 2) + 1) ) {
			Input[ch] = (mHiByte << 24) | (m3rdByte << 16) | (m2ndByte << 8) | mLoByte;
			break;
		}
		else{
			mReadCnt++;
		}
	}
	if(mReadCnt >= BYTE_CONSIST_ATTEMPTS){
		mStatus = -1;
		return mStatus;
	}

	return mStatus;
}

