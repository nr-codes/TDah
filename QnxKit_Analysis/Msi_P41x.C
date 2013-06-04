#include	<stdio.h>
#include <hw/inout.h>
#include "Msi_P41x.h"

#define CONFIG_MASK	0x40
#define STATUS_ADDR	8

Msi_P41x::Msi_P41x(int nCh, double *in, unsigned char *conf, int chOffset){
	Initialized = 0;

	Input = in;
	numCh = nCh;
	numBanks = nCh / 8;
	ChannelConfig = conf;
	ChannelOffset = chOffset;
}

Msi_P41x::~Msi_P41x(){

}

int Msi_P41x::Init(int base){
	Base = base;

	// Configure all channels to +/- 10V by default

	for(curCh = 0; curCh < numCh; curCh++){
		ChannelConfig[curCh] = Msi_P41x::BIPOLAR_10;
	}

	Initialized = 1;
	return 1;
}

int Msi_P41x::ConfigChannel(int ch, ChannelRange range){
	ChannelConfig[ch - ChannelOffset] = range;

	return 1;
}

// The analog io board is made up of n banks 8 channels each.  For example, Msi_P412 has 32 channels on 4 banks.
// Each analog to digital conversion is initiated with a write to a control register; the board is then polled
// for the status register to indicate that conversion has completed.  Doing this sequentially, ie initiating
// a conversion for each channel and waiting for it to be done takes n * tau seconds where n is the number
// of channels and tau is conversion time.  The more channels (or boards) the longer we'll have to wait.
// A more efficient way is to initiate parallel conversions on all banks one channel at a time which leads
// to a total wait time of 8 * tau regardless of the number of channels (boards) present.  A boardwide (all bank)
// conversion on a channel is initiated with a StartConv(ch) function.  ReadBankChannel() function waits
// for conversion to be done and scales the results.

int Msi_P41x::StartConv(int bankChannel){
	for(curBank = 0; curBank < numBanks; curBank++){
		// setup range and start conversion
		out8(Base + (curBank << 1), CONFIG_MASK | ChannelConfig[bankChannel + (curBank << 3)] | bankChannel);
	}
}

int Msi_P41x::ReadBankChannel(int bankChannel){
	if(!Initialized){
		return -1;
	}

	for(curBank = 0; curBank < numBanks; curBank++){
		// wait for converion to finish
		convCount = 0;
		while( in8(Base + STATUS_ADDR) & (0x1 << curBank) ){
			if(++convCount > 1000){
				//puts("sup");
				return -1;
			}
		}

		// read data and convert into proper range
		rawData  =  in8(Base + (curBank << 1));
		rawData |= (in8(Base + (curBank << 1) + 1) & 0xF) << 8;

		curCh = bankChannel + (curBank << 3);
		
		switch(ChannelConfig[curCh]){
			case Msi_P41x::UNIPOLAR_5:
				Input[curCh] = (double)rawData / 4096. * 5.0;
				break;
			case Msi_P41x::UNIPOLAR_10:
				Input[curCh] = (double)rawData / 4096. * 10.0;
				break;
			case Msi_P41x::BIPOLAR_5:

				if(rawData & 0x800){
					rawData = -((~rawData + 1) & 0xFFF);
				}
				Input[curCh] = (double)rawData / 2048. * 5.0;
				break;
			case Msi_P41x::BIPOLAR_10:
				if(rawData & 0x800){
					rawData = -((~rawData + 1) & 0xFFF);
				}
				Input[curCh] = (double)rawData / 2048. * 10.0;
				break;

			default:
				return -1;
		}
	}
	return 1;
}

