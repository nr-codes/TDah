#include <fcdynamic.h>
#include <string.h>

// TODO
// fix parser

#define TIMING 0
#define RECORD 0

// TRACKCAM
#define EXPOSURE 30
#define FRAME_TIME 100
#define IMG_WIDTH 1024
#define IMG_HEIGHT IMG_WIDTH

// TIMER
#define NUM_IMGS 100
#define WIDTH_STEP 2
#define MIN_WIDTH 128
#define MAX_WIDTH 128
#define MIN_FRAME 10
#define MAX_FRAME 100000 // us (10 fps)
#define FRAME_STEP 1000
#define EXPOSURE_STEP 4
#define DURATION 5

// INITIAL BLOB POSITION IN IMG COORD FRAME
#define INITIAL_ROI_X 0
#define INITIAL__ROI_Y 0
#define BOUNDING_BOX IMG_WIDTH
#define INITIAL_BLOB_XMIN 0
#define INITIAL_BLOB_YMIN 0
#define INITIAL_BLOB_WIDTH 1024
#define INITIAL_BLOB_HEIGHT 1024
#define THRESHOLD 128

#define SEQ {ROI_0, ROI_5}
#define SEQ_LEN 2

void initial_blob_positions(TrackingWindow *win)
{
	win[ROI_0].blob_xmin = INITIAL_BLOB_XMIN;
	win[ROI_0].blob_ymin = INITIAL_BLOB_YMIN;
	win[ROI_0].blob_xmax = INITIAL_BLOB_XMIN + INITIAL_BLOB_WIDTH;
	win[ROI_0].blob_ymax = INITIAL_BLOB_YMIN + INITIAL_BLOB_HEIGHT;

	win[ROI_5].blob_xmin = INITIAL_BLOB_XMIN;
	win[ROI_5].blob_ymin = INITIAL_BLOB_YMIN;
	win[ROI_5].blob_xmax = INITIAL_BLOB_XMIN + INITIAL_BLOB_WIDTH;
	win[ROI_5].blob_ymax = INITIAL_BLOB_YMIN + INITIAL_BLOB_HEIGHT;
}

void reset(TrackingWindow *win, int roi_box, double frame, double exposure)
{
	int i;
	int img_w, img_h;
	int blob_cx, blob_cy;
	IplImage *img = NULL;

	memset(win, 0, sizeof(TrackingWindow) * MAX_ROI);
	initial_blob_positions(win);

#if ONLINE
	img_w = IMG_WIDTH;
	img_h = IMG_HEIGHT;
#else
	GetNextImage(&img, 0, ANIMATION_NAME, ANIMATION_LENGTH, FALSE);
	img_w = img->width;
	img_h = img->height;
	cvReleaseImage(&img);
#endif

	for(i = 0; i < MAX_ROI; i++) {
		blob_cx = (win[i].blob_xmin + win[i].blob_xmax) / 2;
		blob_cy = (win[i].blob_ymin + win[i].blob_ymax) / 2;

		win[i].roi = i;
		win[i].roi_w = roi_box;
		win[i].roi_h = roi_box;
		win[i].img_w = img_w;
		win[i].img_h = img_h;

		set_roi_box(win + i, blob_cx, blob_cy);
		fix_blob_bounds(win + i);

#if ONLINE
		SetTrackCamParameters(win + i, frame, exposure);
#endif
	}
}

int capture_video(TrackingSequence *tseq, int num_imgs)
{
	int rc, img_nr, total_imgs;
	IplImage *cvDisplay = NULL;
	Fg_Struct *fg = NULL;
	CvVideoWriter *writer;

	total_imgs = 0;
	img_nr = 1;
	cvDisplay = cvCreateImageHeader(cvSize(IMG_WIDTH, IMG_HEIGHT), 8, 1);
	cvNamedWindow("win", CV_WINDOW_AUTOSIZE);

	writer = cvCreateVideoWriter("out.avi", -1, 1e6 / FRAME_TIME,
							cvSize(IMG_WIDTH,IMG_HEIGHT), FALSE);

	if(writer == NULL) {
		return EINVAL;
	}

	rc = StartGrabbing(&fg, tseq, NULL);
	if(rc != FG_OK) {
		return rc;
	}
	
	while(total_imgs < num_imgs) {
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		cvDisplay->imageData = (char *) Fg_getImagePtr(fg, img_nr, PORT_A);
		cvDisplay->imageDataOrigin = (char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		if(cvDisplay->imageData != NULL) {
			total_imgs++;
			cvWriteFrame(writer, cvDisplay);
			cvShowImage("win", cvDisplay);
			cvWaitKey(20);
		}
		else {
			printf("img is null: %d\n", img_nr);
			break;
		}
	}

	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseImageHeader(&cvDisplay);

	return FG_OK;
}

int main(int argc, char *argv[])
{
	int rc = FG_OK;
	TrackingSequence tseq;
	double frame = 0, exposure = 0, exp_step = 0;
	int box = 0;
	int seq[] = SEQ;

	tseq.seq = (int *) &seq;
	tseq.seq_len = SEQ_LEN;

#if (ONLINE && RECORD)
	reset(tseq.windows, BOUNDING_BOX, FRAME_TIME, EXPOSURE);
	return capture_video(&tseq, 100);
#endif

#if TIMING == 1
	box = MIN_WIDTH;
	while(box <= MAX_WIDTH) {
	#if ONLINE
		for(frame = MAX_FRAME; frame >= MIN_FRAME; frame /= FRAME_STEP) {
			if(frame == MIN_FRAME) {
				exp_step = EXPOSURE_STEP;
			}
			else {
				exp_step = (frame - MIN_FRAME) / EXPOSURE_STEP;
			}

			for(exposure = MIN_FRAME; exposure <= frame; exposure += exp_step) {
					reset(tseq.windows, box, frame, exposure);
					//time_run(&tseq, NUM_IMGS, THRESHOLD, frame, exposure);
					time_run(&tseq, NUM_IMGS, THRESHOLD, FRAME_TIME, EXPOSURE);
			}
		}
	#else
		reset(tseq.windows, box, -1, -1);
		time_run(&tseq, NUM_IMGS, THRESHOLD, -1, -1);
	#endif
		box *= WIDTH_STEP;
	}
#elif TIMING == 2
	box = MIN_WIDTH;
	while(box <= MAX_WIDTH) {
		for(frame = MAX_FRAME; frame >= MIN_FRAME; frame /= FRAME_STEP) {
				reset(tseq.windows, box, frame, EXPOSURE);
				time_dur(&tseq, DURATION, THRESHOLD, frame, EXPOSURE);
		}
		box *= WIDTH_STEP;
	}
#else
	reset(tseq.windows, BOUNDING_BOX, FRAME_TIME, EXPOSURE);
	rc = display_run(&tseq, FRAME_TIME, EXPOSURE);
	if(rc != FG_OK) {
		_getch();
	}
#endif

	return rc;
}
