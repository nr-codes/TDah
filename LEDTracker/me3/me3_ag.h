#ifndef ME3_AG_H_
#define ME3_AG_H_

// includes
#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"

// constants
#if defined(WIN32) && defined(_WIN32)
	#define AG_APPLET "SingleAreaGray.dll"
#endif

#define TIMEOUT 2
#define AG_MAX_WIDTH 1024
#define AG_MAX_HEIGHT 1024
#define me3_err(msg) printf("me3::%s: (%d) %s\n", (msg), Fg_getLastErrorNumber(fg), \
							Fg_getLastErrorDescription(fg));

// functions
extern int me3_ag_init(Fg_Struct **grabber, int trig, int memsize, int nbufs);
extern int me3_ag_acquire(Fg_Struct *fg);
extern int me3_ag_deinit(Fg_Struct *fg);
extern const unsigned long *get_mem();

extern int roi_window(Fg_Struct *fg, int x, int width, int y, int height);
extern int roi_exposure(Fg_Struct *fg, double exp, double ft);

#endif /* ME3_AG_H_ */