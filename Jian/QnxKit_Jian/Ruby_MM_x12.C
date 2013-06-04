#include<stdio.h>
#include <hw/inout.h>

#include "Ruby_MM_x12.h"

#define LSB_REG   		0
#define MSB_REG			1
#define CH_REG          2
#define UPDATE_ALL_REG 	0

Ruby_MM_x12::Ruby_MM_x12(){
	mInitialized = 0;
}

Ruby_MM_x12::~Ruby_MM_x12(){

}

int Ruby_MM_x12::Init(int base){
	mBase = base;

	for(mCurCh = 0; mCurCh < DACBOARD_CH; mCurCh++){ //DACBOARD_CH 8
		Output[mCurCh] = 0.0;
	}

	mInitialized = 1;

	Reset(); // nothing?

	return 1;
}

void Ruby_MM_x12::Reset(){
}

void Ruby_MM_x12::Limit(int ch){
		if(Output[ch] > 10.)
			Output[ch] = 10.;
		else if(Output[ch] < -10.)
			Output[ch] = -10.;
}

int Ruby_MM_x12::WriteAll(){

	if(!mInitialized)
		return -1;

	for(mCurCh = 0; mCurCh < DACBOARD_CH; mCurCh++){

		Limit(mCurCh);

		mCount = (int)((Output[mCurCh]/10. * 2048.) + 2048. + 0.5);
		if(mCount > 4095)
			mCount = 4095;

		// Write LSB
		out8(mBase + LSB_REG, mCount & 0xFF);
		// Select Channel
		out8(mBase + CH_REG, mCurCh);
		// Write MSB
		out8(mBase + MSB_REG, (mCount >> 8) & 0xF);
	}

	// Update all outputs
	in8(mBase + UPDATE_ALL_REG);

	return 1;
}

int Ruby_MM_x12::WriteCh(int ch){
	if(!mInitialized)
		return -1;
		
	if((ch < 0 || ch >= DACBOARD_CH))
		return -1;

	Limit(ch);

	mCount = (int)((Output[ch]/10. * 2048.) + 2048. + 0.5);
	if(mCount > 4095)
		mCount = 4095;
	
	// Write LSB
	out8(mBase + LSB_REG, mCount & 0xFF);
	// Select Channel
	out8(mBase + CH_REG, ch);
	// Write MSB
	out8(mBase + MSB_REG, (mCount >> 8) & 0xF);
	
	// Update all outputs
	in8(mBase + UPDATE_ALL_REG);

	return 1;
}
