#ifndef ME3_H_
#define ME3_H_

// includes
#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

// constants
#if defined(WIN32) && defined(_WIN32)
	#define FC_APPLET "FastConfig.dll"
#endif

#define DO_INIT 1
#define MAX_ROI 8
#define FC_MAX_WIDTH 1024
#define FC_MAX_HEIGHT 1024
#define TIMEOUT 2
#define me3_err(msg) printf("me3::%s: (%d) %s\n", (msg), Fg_getLastErrorNumber(fg), \
							Fg_getLastErrorDescription(fg));

enum roi_index {ROI_0 = 0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

// functions
extern int me3_init(Fg_Struct **grabber, char *applet, int memsize, int buffers);
extern int me3_acquire(Fg_Struct *fg, int *sequence, int seq_len);
extern int me3_deinit(Fg_Struct *fg);
extern const unsigned long *get_mem();

extern int roi_sequence(Fg_Struct *fg, int *seq, int len);
extern int set_roi(int index, int width, int height, int exposure, int frame);
extern int roi_window(int index, int x, int width, int y, int height);
extern int roi_exposure(int index, double exp, double ft);
extern int roi_linlog(int index, int use_linglog, int ll1, int ll2, int comp);
extern int write_roi(Fg_Struct *fg, int index, int tag, int doInit);

#endif /* ME3_H_ */