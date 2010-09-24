/**
* @file utils.cpp contains functions for initiatlizing, acquiring, and deinitializing the 
* frame grabber and camera.
*/

#include <string.h>
#include "fcdynamic.h"

#define NUM_BUFFERS 16
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)

#define NO_LINLOG 0
#define USELINLOG NO_LINLOG
#define LINLOG1 0
#define LINLOG2  0
#define COMP 0
#define CAMLINK FG_CL_DUALTAP_8_BIT

#define NOT_APPLICABLE -1
#define TABLE_ENTRIES 19
#define BUFFER_LEN 100

static char *table_format[] = {
	"image number",
	"region of interest number",
	"performance counter timestamp (counts)",
	"frame grabber timestamp (microseconds)",
	"Fg_getLastPicture start (counts)",
	"Fg_getLastPicture stop (counts)",
	"threshold start (counts)",
	"threshold stop (counts)",
	"blob start (counts)",
	"blob stop (counts)",
	"roi x (pixels)",
	"roi y (pixels)",
	"roi w (pixels)",
	"roi h (pixels)",
	"blob x (pixels)",
	"blob y (pixels)",
	"blob w (pixels)",
	"blob h (pixels)",
	"blob found (true = 1/false = 0)"
};

static char anim_buffer[BUFFER_LEN];

/**
* Grabs the n-th image from file
*
* GrabNextImage is meant to be an alternative to Fg_getLastPicNumberBlocking (see Silicon
* Software SDK doc).  The behavior is similar to Fg_getLastPicNumberBlocking.  The only
* difference is that the images are loaded from disk as oppose to being grabbed from the
* camera.
*
* There are a few particular things to keep in mind about this function call.  First, 
* similarly to Fg_getLastPicNumberBlocking, the desired image number (<code>nr</code>)
* can be any non-negative value.  If <code>nr</code> is greater than the number of frames
* on disk, then the <code>nr</code> % <code>seq_len</code> is returned.

* The name of the image format must be <code>name</code>x.jpg, where <code>name</code> is 
* the base file name of a jpeg image file and x is the x-th image in the sequence.  The 
* first image must have x = 0 and the last image must have x = <code>seq_len</code> - 1.
* Also, x must contain all integer values between 0 and <code>seq_len</code> - 1.  
*
* Images are loaded by counting up from 0 to <code>seq_len</code> - 1 and then counting
* back down to 0.  This process is repeated indefinitely.  If a different base name is
* used GrabNextImage will not reset itself and automatically count up.  Instead, it will 
* continue counting in the same direction as its previous call.  To change directions 
* set <code>nr</code> = 0.
*
* Finally, <code>img</code> does not have to be uninitialized.  If <code>img</code> 
* already points to a preexiting image, then the image will be released prior to 
* loading the next image in the sequence.
*
* @param img the destination where the image will be loaded into
* @param nr the desired image to load
* @param name the base name of the jpeg file
* @param seq_len the length of the sequence on disk
* @param show_name if true, displays the name of the loaded jpeg file
*/

void GetNextImage(IplImage **img, int nr, char *name, int seq_len, int show_name)
{
	static int count_down = 0;

	nr %= seq_len;
	if(nr == 0) {
		count_down = !count_down;
	}

	if(count_down) {
		nr = seq_len - nr;
	}
	else {
		nr++;
	}

	memset(anim_buffer, 0, sizeof(anim_buffer));
	sprintf_s(anim_buffer, sizeof(anim_buffer), "%s%d.jpg", name, nr);
	cvReleaseImage(img);
	*img = cvLoadImage(anim_buffer, CV_LOAD_IMAGE_GRAYSCALE);

	if(show_name){
		printf("%s\n", anim_buffer);
	}
}

/**
* Combines routine startup code prior to writing the region of interest to the camera.
*
* SetTrackCamParameters will copy the desired position, window size, frame, and exposure
* time into the internal cam.cpp ROI structure.  It will also set the rarely used linlog
* parameters (see Silicon Software doc) to <code>USELINLOG, LINLOG1, LINLOG2, COMP</code>.
* 
* @param win the tracking window with <code>win->roi, win->roi_xoff, win->roi_w, 
* win->roi_yoff, win->roi_h</code> set to reasonable values (see note below)
* @param frame the frame time in microseconds (i.e. length of time between images)
* @param exposure the exposure time in microseconds (i.e. length of time camera shutter is
* open)
*
* @note reasonable values are: 0 < roi < 8, roi_xoff % 4 = 0 && roi_xoff <= 1024, 
* roi_w % 4 = 0 && roi_w > 4, roi_yoff <= 1024, and roi_h <= 1024.
*/

int SetTrackCamParameters(TrackingWindow *win, double frame, double exposure)
{
	int rc;

	rc = roi_window(win->roi, win->roi_xoff, win->roi_w, win->roi_yoff, win->roi_h);
	if(rc != FG_OK) {
		printf("main: invalid window parameter(s).\n");
		return rc;
	}

	rc = roi_exposure(win->roi, exposure, frame);
	if(rc != FG_OK) {
		printf("main: invalid exposure parameter(s).\n");
		return rc;
	}

	rc = roi_linlog(win->roi, USELINLOG, LINLOG1, LINLOG2, COMP);
	if(rc != FG_OK) {
		printf("main: invalid linlog parameter(s).\n");
		return rc;
	}

	return FG_OK;
}

/**
* copies image data from an image to the tracking window
*
* CopyImageToTrackingWindow takes an image (typically from disk) and copies the image
* data to the tracking window.  The data is expected to be an 8-bit gray scale image
* with at least enough pixels to fill up the tracking window's region of interest.  In
* other words there must be roi_w x roi_h bytes in the image starting at location
* (roi_xoff, roi_yoff).
*
* @param win the destination TrackingWindow
* @param img the source image
*/

void CopyImageToTrackingWindow(TrackingWindow *win, IplImage *img)
{
	int i, j;
	int xmin, xmax, ymin, ymax;
	int W;

	xmin = win->roi_xoff;
	ymin = win->roi_yoff;
	xmax = xmin + win->roi_w;
	ymax = ymin + win->roi_h;

	W = img->width;

	for(i = ymin; i < ymax; i++) {
		for(j = xmin; j < xmax; j++) {
			PIXEL(win, i - ymin, j - xmin) = img->imageData[j + (i * W)];
		}
	}
}

/**
* copies image data from the tracking window to an image
*
* CopyTrackingWindowToImage takes the image data stored in the TrackingWindow and copies 
* the image data to the image.  The image buffer in <code>img</code> must be able to hold
* an 8-bit gray scale image with roi_w x roi_h bytes starting at location
* (roi_xoff, roi_yoff) in the <code>img</code>.
*
* @param win the source TrackingWindow
* @param img the destination image
*/

void CopyTrackingWindowToImage(TrackingWindow *win, IplImage *img)
{
	int i, j;
	int xmin, xmax, ymin, ymax;
	int W;

	xmin = win->roi_xoff;
	ymin = win->roi_yoff;
	xmax = xmin + win->roi_w;
	ymax = ymin + win->roi_h;

	W = img->width;

	for(i = ymin; i < ymax; i++) {
		for(j = xmin; j < xmax; j++) {
			img->imageData[j + (i * W)] = PIXEL(win, i - ymin, j - xmin);
		}
	}
}

/**
* prints a summary of the timing tests performed on the vision system.
*
* PrintTimingData is meant to be used in conjunction with the TimingInfo structure.
* It will print out a summary of the parameters used in the vision system and a table
* of timing data for different aspects of it.  If images were not grabbed from the TrackCam
* camera set fg = NULL.
*
* @param fg the frame grabber structure (see Silicon Software SDK doc), which can be NULL
* @param timer the data structure containing all of the relevant timing information
*
* @see time_run.cpp
* @see TimingInfo
*/

void PrintTimingData(Fg_Struct *fg, TimingInfo *timer)
{
	static int run = 0;
	int i, rc = 0;
	FrameInfo *frame = timer->frame;

	if(run == 0) {
		// print summary info
		printf("======================= SUMMARY =======================\n");
		printf("data collected by Moments compiled %s @ %s\n", __DATE__, __TIME__);
		for(i = 0; i < TABLE_ENTRIES; i++) {
			printf("%s\t", table_format[i]);
		}
		printf("\n\n");
	}

	run++;
	// print run info
	printf("======================= RUN %d =======================\n", run);
	printf("number of images: %d\n", timer->num_imgs);
	if(fg != NULL) {
		printf("number of grabbed images: %d\n", 
			Fg_getStatus(fg, NUMBER_OF_GRABBED_IMAGES, 0, PORT_A));
		printf("number of lost images: %d\n", 
			Fg_getStatus(fg, NUMBER_OF_LOST_IMAGES, 0, PORT_A));
		printf("number of images in progress: %d\n", 
			Fg_getStatus(fg, NUMBER_OF_IMAGES_IN_PROGRESS, 0, PORT_A));
		printf("number of recently acquired image: %d\n",
			Fg_getStatus(fg, NUMBER_OF_ACT_IMAGE, 0, PORT_A));
		printf("number of last get image: %d\n", 
			Fg_getStatus(fg, NUMBER_OF_LAST_IMAGE, 0, PORT_A));
		printf("number of next get image: %d\n", 
			Fg_getStatus(fg, NUMBER_OF_NEXT_IMAGE, 0, PORT_A));
	}
	else {
		// for compatibility with parsers
		printf("number of grabbed images: %d\n", NOT_APPLICABLE);
		printf("number of lost images: %d\n", NOT_APPLICABLE);
		printf("number of images in progress: %d\n", NOT_APPLICABLE);
		printf("number of recently acquired image: %d\n", NOT_APPLICABLE);
		printf("number of last get image: %d\n", NOT_APPLICABLE);
		printf("number of next get image: %d\n", NOT_APPLICABLE);
	}

	printf("start time: %lld counts\n", timer->loop_start.QuadPart);
	printf("stop time: %lld counts\n", timer->loop_stop.QuadPart);
	printf("resolution of performance counter: %lld counts/sec\n", timer->freq.QuadPart);
	printf("performance counter maximum value: %lld counts\n", LLONG_MAX);
	printf("frame grabber maximum value: %lld us\n", ULONG_MAX);

	printf("frame time: %g us\n", timer->roi_f);
	printf("exposure time: %g us\n", timer->roi_e);
	printf("width: %d pixels\n", timer->roi_w);
	printf("height: %d pixels\n", timer->roi_h);

	// print frame data
	printf("\n");
	for(i = 0; i < timer->num_imgs; i++) {
		printf("%d\t", frame[i].img);
		printf("%d\t", frame[i].win.roi);
		printf("%lld\t", frame[i].pc_ts.QuadPart);
		if(fg != NULL) {
			printf("%lld\t", frame[i].fg_ts);
		}
		else {
			printf("%d\t", NOT_APPLICABLE);
		}
		printf("%lld\t", frame[i].grab_start.QuadPart);
		printf("%lld\t", frame[i].grab_stop.QuadPart);
		printf("%lld\t", frame[i].thresh_start.QuadPart);
		printf("%lld\t", frame[i].thresh_stop.QuadPart);
		printf("%lld\t", frame[i].blob_start.QuadPart);
		printf("%lld\t", frame[i].blob_stop.QuadPart);
		printf("%d\t%d\t%d\t%d\t", 
			frame[i].win.roi_xoff, 
			frame[i].win.roi_yoff, 
			frame[i].win.roi_w, 
			frame[i].win.roi_h);
		printf("%d\t%d\t%d\t%d\t",
			frame[i].win.blob_xmin, 
			frame[i].win.blob_ymin, 
			frame[i].win.blob_xmax - frame[i].win.blob_xmin, 
			frame[i].win.blob_ymax - frame[i].win.blob_ymin);
		printf("%d\n", frame[i].blob_found == OBJECT_FOUND);
	}

	// add last new line
	printf("\n");
}

/**
* initialization routines for initializing and grabbing camera (or disk) images
*
* StartGrabbing is a wrapper routine for typical startup code when grabbing from a 
* camera or disk.  If <code>ONLINE</code> is true then <code>fg</code> must not be NULL,
* otherwise if <code>ONLINE</code> is false, then <code>data</code> must not be NULL.
*
* @param fg a pointer to the Silicon Software frame grabber structure (see Silicon
* Software SDK doc) [if ONLINE = 1]
* @param tseq the sequence in which the ROI are active (if ONLINE = 1)
* @param data the buffer that will eventually hold the image data (if ONLINE = 0)
*/

int StartGrabbing(Fg_Struct **fg, TrackingSequence *tseq, unsigned char **data)
{
	int rc = FG_OK;
	TrackingWindow *win = tseq->windows + tseq->seq[0];

#if ONLINE
	rc = init_cam(fg, MEMSIZE(win->roi_w, win->roi_h), NUM_BUFFERS, CAMLINK);
	if(rc != FG_OK) {
		printf("init: %s\n", Fg_getLastErrorDescription(*fg));
		Fg_FreeGrabber(*fg);
		return rc;
	}

	rc = acquire_imgs(*fg, tseq->seq, tseq->seq_len);
	if(rc != FG_OK) {
		printf("init: %s\n", Fg_getLastErrorDescription(*fg));
		Fg_FreeGrabber(*fg);
		return rc;
	}
#else
	*data = (unsigned char *) calloc(win->roi_w * win->roi_h, sizeof(unsigned char));
	if(*data == NULL) {
		printf("main: not enough memory to allocate data.\n");
		return ENOMEM;
	}
#endif

	return FG_OK;
}
