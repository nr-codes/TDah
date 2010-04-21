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

int time_run(TrackingSequence *tseq, int num_imgs, int t, double frame, double exposure)
{
	int rc;
	int img_nr, prev_nr, total_imgs;
	int cur_win;
	TrackingWindow *cur;
	TimingInfo timer;

#if ONLINE
	Fg_Struct *fg = NULL;
#else
	IplImage *faux_fg = NULL;
	unsigned char *data = NULL;
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

	memset(&timer, 0, sizeof(TimingInfo));
	timer.num_imgs = num_imgs;
	timer.roi_e = exposure;
	timer.roi_f = frame;
	timer.roi_w = cur->roi_w;
	timer.roi_h = cur->roi_h;
	timer.frame = (frame_info *) calloc(num_imgs, sizeof(FrameInfo));
	if(timer.frame == NULL) {
		return ENOMEM;
	}
	if(!QueryPerformanceFrequency(&timer.freq)) {
		printf("main: no perfmance counter\n");
		return ENODEV;
	}

#if ONLINE
	rc = StartGrabbing(&fg, tseq, NULL);
#else
	rc = StartGrabbing(NULL, tseq, &data);
#endif
	if(rc != FG_OK) {
		return rc;
	}

	// start image loop
	QueryPerformanceCounter(&timer.loop_start);
	while(total_imgs < num_imgs) {
		cur = tseq->windows + tseq->seq[cur_win];
		cur_win++;
		cur_win %= tseq->seq_len;
#if ONLINE
		QueryPerformanceCounter(&(timer.frame[total_imgs].grab_start));
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		cur->img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
		QueryPerformanceCounter(&(timer.frame[total_imgs].grab_stop));
#else
		GetNextImage(&faux_fg, img_nr, ANIMATION_NAME, ANIMATION_LENGTH, FALSE);
		cur->img = data;
		CopyImageToTrackingWindow(cur, faux_fg);
		img_nr++;
#endif

		if(cur->img != NULL) {
			// process image
			QueryPerformanceCounter(&(timer.frame[total_imgs].thresh_start));
			threshold(cur, t);
			QueryPerformanceCounter(&(timer.frame[total_imgs].thresh_stop));
			
			QueryPerformanceCounter(&(timer.frame[total_imgs].blob_start));
			rc = position(cur);
			QueryPerformanceCounter(&(timer.frame[total_imgs].blob_stop));

#if ONLINE
			write_roi(fg, cur->roi, img_nr, !DO_INIT);
#endif

			// record state
			if(prev_nr != img_nr || img_nr < FG_OK) {
				QueryPerformanceCounter(&(timer.frame[total_imgs].pc_ts));
				timer.frame[total_imgs].img = img_nr;
				timer.frame[total_imgs].blob_found = rc;
				memcpy(&(timer.frame[total_imgs].win), cur, sizeof(TrackingWindow));
#if ONLINE
				// note: added after the fact, might cause delays (doubtful tho')
				timer.frame[total_imgs].fg_ts = img_nr;
				rc = Fg_getParameter(fg, FG_TIMESTAMP, 
					&(timer.frame[total_imgs].fg_ts), PORT_A);
				if(rc != FG_OK) {
					timer.frame[total_imgs].fg_ts = rc;
				}
#endif
				prev_nr = img_nr;
				total_imgs++;
			}
		}
		else {
			printf("img is null: %d\n", img_nr);
			break;
		}
	}
	QueryPerformanceCounter(&timer.loop_stop);

#if ONLINE
	PrintTimingData(fg, &timer);

	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	if(cur->img == NULL) {
		return !FG_OK;
	}
#else
	PrintTimingData(NULL, &timer);

	cvReleaseImage(&faux_fg);
	free(data);
#endif
	free(timer.frame);

	return FG_OK;
}
