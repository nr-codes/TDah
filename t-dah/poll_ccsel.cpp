/**
* @file time_run.cpp a profiling program that records the length of time of certain 
* functions of interest.
*/

#include "fcdynamic.h"

/**
* times different parts of the software
*
* <code>time_run</code> acquires <code>num_imgs</code> and times several functions in a 
* tight loop.  At the end of each run the results are printed to stdout, which can then
* be piped to a file or another program for further processing.
*
* @param tseq the TrackingSequence specifying the active ROIs and their initial positions
* in the image prior to tracking an object
* @param num_imgs the number of images to collect
* @param t the threshold value to be used with <code>threshold</code>
* @param frame the frame time (e.g. length of time between images) in microseconds
* @param exposure the exposure time (e.g. length of time the shutter is kept open) in
* microseconds
*/

int poll_ccsel(TrackingSequence *tseq, int num_imgs, int t, double frame, double exposure)
{
	int rc;
	int img_nr, prev_nr, total_imgs;
	int cur_win;
	TrackingWindow *cur;
	LARGE_INTEGER exp_s, fc_s, fc_e, freq;

#if ONLINE
	Fg_Struct *fg = NULL;
#else
#error "poll_ccsel not supported offline, because ccsel is meaningless. \
	Please recompile with ONLINE = 1 in order to use poll_ccsel."
#endif

	if(tseq->seq_len < MIN_SEQ_LEN) {
		printf("invalid sequence length...need at least %d windows\n", MIN_SEQ_LEN);
		return EINVAL;
	}

	// initialize parameters
	rc = FG_OK;
	cur_win = 0;
	img_nr = 1;
	prev_nr = 0;
	total_imgs = 0;
	cur = tseq->windows + tseq->seq[cur_win];

	if(!QueryPerformanceFrequency(&freq)) {
		printf("main: no perfmance counter\n");
		return ENODEV;
	}

	rc = StartGrabbing(&fg, tseq, NULL);
	if(rc != FG_OK) {
		return rc;
	}

	// start image loop	
	while(total_imgs < num_imgs) {
		//QueryPerformanceCounter(&timer.loop_start);
		Fg_getParameter
		//QueryPerformanceCounter(&timer.loop_stop);
	}

	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	return FG_OK;
}
