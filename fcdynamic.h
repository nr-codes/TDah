/**
* @file fcdynamic.h a required header file
*/

#ifndef FCDYNAMIC_H_
#define FCDYNAMIC_H_

// includes
#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <conio.h>
#include <errno.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

// constants
/**
* determines whether to use code meant for a live camera or images from a file
*
* ONLINE is a debugging switch for testing the system on images (ONLINE == 0) or
* is connected to the Photonfocus TrackCam camera (ONLINE != 0)
*/
#define ONLINE 1

#define MIN_SEQ_LEN 2

#define ANIMATION_LENGTH 32
#define ANIMATION_NAME "cross/Slide"

#define WHITE 255
#define GRAY 128
#define BLACK 0
#define BACKGROUND BLACK
#define FOREGROUND WHITE
#define BORDER GRAY

#define OBJECT_FOUND 0

#define TIMEOUT 5
#define DO_INIT 1
#define MAX_ROI 8 /* limited by FastConfig Applet (see meIII documentation) */

/** 
* the eight indices enumerated as ROI_n
*
* roi_index is an simple way to refer to the 8 ROIs that the Silicon Software API
* allows.
*/
enum roi_index {ROI_0 = 0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

// macros
/**
* set or get the pixel at location [<code>i</code>, <code>j</code>] in the 
* tracking_window <code>win</code> pixel array.
*
* @note [<code>i</code>, <code>j</code>] is considered in the ROI reference frame.
*/

#define PIXEL(win, i, j) ((win)->img[(j) + ((i) * (win)->roi_w)])

// variables and types

/** 
* Keeps updated state information on the position of the ROI and the object
* being tracked.
*
* a <code>tracking_window</code> is most commonly used with functions that are interested
* in the position information of the object (blob) and the ROI.  These functions are 
* typically image processing or tracking routines used to update future positions of the
* ROI.  If extending the vision system to include more image processing or tracking
* routines expect to use this structure often.
*
* @note the ROI and the object coordinates are in two different reference frames.  The ROI
* is in the image reference frame (e.g 0 <= roi_x <= img_w) while the object is in the
* ROI reference frame (e.g. 0 <= blob_xmin <= roi_w).
*
* @see PIXEL
* @see threshold
* @see position
* @see blob
*/

struct tracking_window {
	int roi; /**< the index of the ROI */
	int roi_xoff; /**< the ROI x coordinate in the img_w x img_h image reference frame */
	int roi_yoff; /**< the ROI y coordinate in the img_w x img_h image reference frame */
	int roi_w; /**< the ROI width */
	int roi_h; /**< the ROI height */
	
	int blob_xmin; /**< the object's uppermost x coordinate in the ROI reference frame */
	int blob_ymin; /**< the object's uppermost y coordinate in the ROI reference frame */
	int blob_xmax; /**< the object's lowermost x coordinate in the ROI reference frame */
	int blob_ymax; /**< the object's lowermost y coordinate in the ROI reference frame */
	
	int img_w; /**< the image's total width */
	int img_h; /**< the image's total height */
	unsigned char *img; /**< point to the grayscale 8-bit image data */
};

/**
* the shorthand and actually used form of tracking_window
*
* @see tracking_window
*/

typedef struct tracking_window TrackingWindow;

/**
* A convenience structure containing the active ROIs and the order in which the 
* ROIs are activated.
*
* tracking_sequence is meant to make tracking multiple objects easier by reducing the
* number of parameters that need to be passed around.  This structure is typically used
* in functions that need to update multiple ROIs and the helper functions that are
* associated with managing multiple ROIs.
*
* @see time_run.cpp or display_run.cpp
* @see StartGrabbing
*/

struct tracking_sequence {
	int *seq;
	int seq_len;

	TrackingWindow windows[MAX_ROI];
};

/**
* the shorthand and actually used form of tracking_sequence
*
* @see tracking_window
*/

typedef struct tracking_sequence TrackingSequence;

/**
* timing information for a particular frame
*
* frame_info is used for timing segments of code on a frame by frame basis.  
* An example use of this structure can be found in time_run.cpp.
*
* @note if this structure is extended in the future by adding more timing parameters 
* make sure to modify the functions and variables in utils.cpp in order to have the
* information printed to the screen.
*
* @see utils.cpp
* @see time_run.cpp
* @see PrintTimingData
*/

struct frame_info {
	int img;
	int blob_found;
	TrackingWindow win;
	LARGE_INTEGER grab_start;
	LARGE_INTEGER grab_stop;
	LARGE_INTEGER thresh_start;
	LARGE_INTEGER thresh_stop;
	LARGE_INTEGER blob_start;
	LARGE_INTEGER blob_stop;
	LARGE_INTEGER pc_ts;
	__int64 fg_ts;
};

typedef struct frame_info FrameInfo;

/**
* Global and local timing information
*
* timing_info maintains information of frame by frame timing statistics and other useful
* information for later analysis of the system's runtime.  An example use of this 
* structure can be found in time_run.cpp.
*
* @see utils.cpp
* @see time_run.cpp
* @see PrintTimingData
*/

struct timing_info {
	int num_imgs;
	int dur;
	double roi_f;
	double roi_e; 
	int roi_w;
	int roi_h;

	LARGE_INTEGER loop_start; 
	LARGE_INTEGER loop_stop;
	LARGE_INTEGER freq;

	FrameInfo *frame;
};

typedef struct timing_info TimingInfo;

// functions
extern int init_cam(Fg_Struct **grabber, int memsize, int buffers, int camlink);
extern int acquire_imgs(Fg_Struct *fg, int *sequence, int seq_len);
extern int deinit_cam(Fg_Struct *fg);
extern const unsigned long *get_mem();

extern int roi_sequence(Fg_Struct *fg, int *seq, int len);
extern int set_roi(int index, int width, int height, int exposure, int frame);
extern int roi_window(int index, int x, int width, int y, int height);
extern int roi_exposure(int index, double exp, double ft);
extern int roi_linlog(int index, int use_linglog, int ll1, int ll2, int comp);
extern int write_roi(Fg_Struct *fg, int index, int imgNr, int doInit);

extern int threshold(TrackingWindow *win, int t);
extern int boundary(TrackingWindow *win);
extern int erode(TrackingWindow *win);

extern int time_run(TrackingSequence *tseq, int num_imgs, int t, double frame, double exposure);
extern int time_dur(TrackingSequence *tseq, int dur, int t, double frame, double exposure);
extern int display_run(TrackingSequence *tseq, double frame, double exposure);

extern void set_roi_box(TrackingWindow *win, int x, int y);
extern void fix_blob_bounds(TrackingWindow *win);
extern void set_region(int e, int x, int y, int flags, void *param);
extern int position(TrackingWindow *cur);
extern int blob(TrackingWindow *win);

extern int open_comm();
extern int write_comm(TrackingWindow *win, int box_x, int box_y);
extern int close_comm();

extern int StartGrabbing(Fg_Struct **fg, TrackingSequence *tseq, unsigned char **data);
extern void CopyTrackingWindowToImage(TrackingWindow *win, IplImage *img);
extern void CopyImageToTrackingWindow(TrackingWindow *win, IplImage *img);
extern void PrintTimingData(Fg_Struct *fg, TimingInfo *timing_info);
extern void GetNextImage(IplImage **img, int nr, char *name, int seq_len, int show_name);
extern int SetTrackCamParameters(TrackingWindow *win, double frame, double exposure);

#endif /* FCDYNAMIC_H_ */