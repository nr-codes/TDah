/**
* @file skeleton.cpp displays code for a simple one ROI tracker
*
* skeleton.cpp contains the necessary components to start and modify the vision system for
* application-specific tracking/image processing tasks.  The code is comprised of three
* functions: set_initial_positions(...), display_tracking(...), and main().
*
* set_initial_positions(...) shows one possible initialization strategy for the camera's
* ROI.  This implementation simply takes the known initial position of the object to track
* (the blob) and centers the camera's ROI bounding box around that center.
*
* display_tracking(...) is a sample application of interest for showing (processed) camera
* frames.  A function like this also provides visual feedback of what is going on 
* internally in the system and is an invaluable debugging tool (see display_run.cpp for
* a more complicated example).
*
* Finally, main() is the starting point of the program.  Its role is to initialize the
* camera and run the logic necessary for tracking objects.
*
* Most, if not all, applications will follow this recipe of initialization routines,
* application-specific routines, and finally a main program that sits in a loop acquiring
* the next image to process.  A few common setup and helper functions can be found in 
* utils.cpp.
*/

#include "fcdynamic.h"

// CAMERA PARAMETERS
#define EXPOSURE 20000 /**< shutter speed in us */
#define FRAME_TIME 50000 /**< pause between images in us (e.g. 1 / fps) */
#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define NUM_BUFFERS 16 /**< typical setting (max is 1,000,000 shouldn't exceed 1.6 GB)*/
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)

#define SEQ {ROI_0}
#define SEQ_LEN 1
#define CAMLINK FG_CL_DUALTAP_8_BIT

// CAMERA REGION OF INTEREST
#define BOUNDING_BOX 1024

// INITIAL BLOB POSITION IN IMG COORD FRAME
#define INITIAL_BLOB_XMIN (322+24)
#define INITIAL_BLOB_YMIN (423+24)
#define INITIAL_BLOB_WIDTH BOUNDING_BOX
#define INITIAL_BLOB_HEIGHT BOUNDING_BOX

// APPLICATION-SPECIFIC PARAMETERS
#define BITS_PER_PIXEL 8
#define NUM_CHANNELS 1
#define THRESHOLD 223
#define DISPLAY "SimpleTracking" /**< name of display GUI */
#define NEXT_IMAGE 2 /**< next valid image to grab */

/** Sets the initial positions of the camera's window and blob's window
*
*  set_initial_position sets the vision systems two most important parameters.  The
*  initial position or "best guess" location of the blob and the camera's ROI.  The
*  camera's ROI determines the size of the image that the camera will send back to 
*  the application and where in the image the window is located.
*
*  Keep in mind that there is in effect two ROIs that are being tracked.  The first ROI, 
*  those prefixed with "roi_" in TrackingWindow are parameters that are eventually sent 
*  to the camera.  These parameters can be considered the hardware ROI.  The hardware 
*  components of the system (i.e. the frame grabber and camera) can only be programmed 
*  via these parameters.
*
*  Because there are limitation with where the hardware-based ROIs can be placed (i.e. 
*  roi_x & roi_w must be multiples of 4 and roi_w should be >= 8 pixels), the software
*  ROI allows for finer control of the image area to inspect.  These variables are prefixed
*  with "blob_".  The software ROI, or blob coordinates, is used by majority of the code 
*  base for object tracking (see imgproc.cpp).
*
*  It is assumed that when "ROI" is used that it refers to the camera's ROI parameters
*  and "blob" refers to the software's parameters.
*/

void set_initial_positions(TrackingWindow *win)
{
	int blob_cx, blob_cy;
	/* The following example shows how to initialize the ROI for the camera ("roi_")
		and object to track ("blob_").  This function can be generalized to include all
		eight ROIs by copying and pasting the code below or by writing a generic loop
	*/

	// insert initial image coordinates of ROI 0 for camera
	win->roi = ROI_0;
	win->roi_w = BOUNDING_BOX;
	win->roi_h = BOUNDING_BOX;
	win->img_w = IMG_WIDTH;
	win->img_h = IMG_HEIGHT;

	// store the camera's ROI 0 information
	SetTrackCamParameters(win + ROI_0, FRAME_TIME, EXPOSURE);

	// insert initial image coordinates of blob 0 (for software use)
	win->blob_xmin = INITIAL_BLOB_XMIN;
	win->blob_ymin = INITIAL_BLOB_YMIN;
	win->blob_xmax = INITIAL_BLOB_XMIN + INITIAL_BLOB_WIDTH;
	win->blob_ymax = INITIAL_BLOB_YMIN + INITIAL_BLOB_HEIGHT;

	// convert from the blob's image coordinate system to the ROI 
	// coordinate system.  This only needs to be done during initialization, 
	// because all routines in the tracking code assume that the blob is 
	// relative to the currently active ROI window and remain in that coordinate 
	// frame.
	fix_blob_bounds(win);

	// center camera's ROI 0 around the blob's midpoint.  Note that in this 
	// implementation the initial placement of the ROI is dependent on the blob's 
	// initial coordinates.
	blob_cx = (win->blob_xmin + win->blob_xmax) / 2;
	blob_cy = (win->blob_ymin + win->blob_ymax) / 2;
	set_roi_box(win, blob_cx, blob_cy);

	// store parameters...note these parameters are NOT sent to the camera
	// they are stored internally, because the Silicon Software doc does not
	// make it clear on how to read what ROI parameters are currently active
	// in the camera.
	//
	// In order to send the coordinates to the camera, it is 
	// required to call write_roi(...) AFTER calling SetTrackCamParameters(...)
	// or any of the individual functions that SetTrackCamParameters(...) relies
	// on.  To summarize, writing to the camera is a two step process:
	//
	//  1) SetTrackCamParameters(win, FRAME_TIME, EXPOSURE); <- buffer parameters internally
	//  2) write_roi(fg, cur.roi, img_nr, !DO_INIT); <- writes buffered parameters to camera
	SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);
}

/** Draw ROI & blob windows and show image on screen (see OpenCV doc for info)
*
* display_tracking simply displays the current frame on screen.
*/

void display_tracking(TrackingWindow *cur, IplImage *gui)
{
	gui->imageData = (char *) cur->img;
	gui->imageDataOrigin = (char *) cur->img;

	// roi box
	cvRectangle(gui, cvPoint(cur->roi_xoff, cur->roi_yoff), 
		cvPoint(cur->roi_xoff + cur->roi_w, cur->roi_yoff + cur->roi_h), 
		cvScalar(128));

	// blob box
	cvRectangle(gui, cvPoint(cur->blob_xmin + cur->roi_xoff, 
		cur->blob_ymin + cur->roi_yoff), 
		cvPoint(cur->blob_xmax + cur->roi_xoff, cur->blob_ymax + cur->roi_yoff), 
		cvScalar(128));

	// show image
	CopyTrackingWindowToImage(&cur, cvDisplay);
	cvShowImage(DISPLAY, gui);

	// add a small delay, so OpenCV has time to display to screen
	cvWaitKey(1);
}

/** Grabs an image from the camera and displays the image on screen
*
*  The purpose of this program is to show how to get the camera up and running.
*  Modifications can be made by modifying the while(...) loop with different image
*  processing and tracking logic.
*/

int main()
{
	// important variables used in most applications
	int rc;
	Fg_Struct *fg = NULL;
	int img_nr;
	TrackingWindow cur;
	int seq[] = SEQ;

	// following lines are for displaying images only!  See OpenCV doc for more info.
	// they can be left out, if speed is important.
	IplImage *cvDisplay = NULL;

	cvDisplay = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), 
		BITS_PER_PIXEL, NUM_CHANNELS);
	cvNamedWindow(DISPLAY, CV_WINDOW_AUTOSIZE);
	
	// initialize the tracking window (i.e. blob and ROI positions)
	memset(&cur, 0, sizeof(TrackingWindow));
	set_initial_positions(&cur);

	// initialize the camera
	rc = init_cam(&fg, MEMSIZE(cur.roi_w, cur.roi_h), NUM_BUFFERS, CAMLINK);
	if(rc != FG_OK) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		Fg_FreeGrabber(fg);
		return rc;
	}

	// start acquiring images (this function also writes any buffered ROIs to the camera)
	rc = acquire_imgs(fg, (int *) &seq, SEQ_LEN);
	if(rc != FG_OK) {
		printf("init: %s\n", Fg_getLastErrorDescription(fg));
		Fg_FreeGrabber(fg);
		return rc;
	}

	// initialize parameters
	img_nr = 1;

	// start image loop and don't stop until the user presses 'q'
	printf("press 'q' at any time to quit this demo.");
	while(cvWaitKey(1) == 'q') {
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		cur.img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		// make sure that camera returned a valid image
		if(cur.img != NULL) {
			// increment to the next desired frame.  This has to be at least
			// +2, because the camera's ROI will not be active until the second
			// frame (see Silicon Software FastConfig doc)
			img_nr += NEXT_IMAGE;

			// process image
			threshold(&cur, THRESHOLD);

			// update ROI position
			position(&cur);

			// at this point position(...) has updated the ROI, but it only stores
			// the updated values internal to the code.  The next step is to flush
			// the ROI to the camera (see position(...) documentation).

			// write ROI position to camera to be updated on frame "img_nr"
			write_roi(fg, cur.roi, img_nr, !DO_INIT);

			// show image on screen
			display_tracking(&cur, cvDisplay);
		}
		else {
			// typically this state only occurs if an invalid ROI has been programmed
			// into the camera (e.g. roi_w == 4).
			printf("img is null: %d\n", img_nr);
			break;
		}
	}

	// free viewer resources
	cvReleaseImage(&cvDisplay);

	// free camera resources
	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	return FG_OK;
}