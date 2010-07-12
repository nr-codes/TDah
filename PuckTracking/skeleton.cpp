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

#include <fcdynamic.h>

// CAMERA PARAMETERS
#define EXPOSURE (30) /**< shutter speed in us */
#define FRAME_TIME (100) /**< pause between images in us (e.g. 1 / fps) */
#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define NUM_BUFFERS 16 /**< typical setting (max is 1,000,000 shouldn't exceed 1.6 GB)*/
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)

#define SEQ {ROI_0, ROI_1}
#define SEQ_LEN 2
#define CAMLINK FG_CL_DUALTAP_8_BIT

// CAMERA REGION OF INTEREST
#define ROI_BOX 1024

// INITIAL BLOB0 POSITION IN IMG COORD FRAME
#define INITIAL_BLOB0_XMIN (391)
#define INITIAL_BLOB0_YMIN (430)
#define INITIAL_BLOB0_WIDTH 25
#define INITIAL_BLOB0_HEIGHT 25

// INITIAL BLOB1 POSITION IN IMG COORD FRAME
#define INITIAL_BLOB1_XMIN (602)
#define INITIAL_BLOB1_YMIN (414)
#define INITIAL_BLOB1_WIDTH 25
#define INITIAL_BLOB1_HEIGHT 25


#define INIT_BLOB_COORD_MIN {INITIAL_BLOB0_XMIN, INITIAL_BLOB1_XMIN, INITIAL_BLOB0_YMIN, INITIAL_BLOB1_YMIN}

#define INIT_BLOB_COORD_MAX {INITIAL_BLOB0_XMIN + INITIAL_BLOB0_WIDTH, INITIAL_BLOB1_XMIN + INITIAL_BLOB1_WIDTH, \
								INITIAL_BLOB0_YMIN + INITIAL_BLOB0_HEIGHT, INITIAL_BLOB1_YMIN + INITIAL_BLOB1_HEIGHT}

// APPLICATION-SPECIFIC PARAMETERS
#define BITS_PER_PIXEL 8
#define NUM_CHANNELS 1
#define THRESHOLD0 12
#define THRESHOLD1 17
#define DISPLAY0 "Simple Tracking 0" /**< name of display GUI */
#define DISPLAY1 "Simple Tracking 1" /**< name of display GUI */
#define NEXT_IMAGE 1 /**< next valid image to grab */
#define PORT TEXT("COM5") /**< name of comm port */
#define SHOW_DISP 1 /**< show display, turn off for accurate timing */
#define TEXT_BUF 100
#define APP_NUM_ROI 2
#define PACKET_SIZE 17

char *disp[] = {DISPLAY0, DISPLAY1};

struct {
	char roi;
	float x;
	float y;
	unsigned int t;
} roi_packet[APP_NUM_ROI];


char packet[PACKET_SIZE*APP_NUM_ROI];

extern void pad_blob_region(TrackingWindow *win);
extern int panic(TrackingWindow *win);

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

void set_initial_positions(TrackingWindow wins[])
{
	int i;
	int blob_cx, blob_cy;
	int seq[] = SEQ;
	int blob_min[] = INIT_BLOB_COORD_MIN;
	int blob_max[] = INIT_BLOB_COORD_MAX;
	TrackingWindow *win;

	/* The following example shows how to initialize the ROI for the camera ("roi_")
		and object to track ("blob_").  This function can be generalized to include all
		eight ROIs by copying and pasting the code below or by writing a generic loop
	*/

	// insert initial image coordinates of ROI for camera
	for(i = 0; i < SEQ_LEN; i++) {
		win = wins + i;

		win->roi = seq[i];
		win->roi_w = ROI_BOX;
		win->roi_h = ROI_BOX;
		win->img_w = IMG_WIDTH;
		win->img_h = IMG_HEIGHT;

		// store the camera's ROI information
		SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);

		// insert initial image coordinates of blob (for software use)
		win->blob_xmin = blob_min[i];
		win->blob_ymin = blob_min[i + SEQ_LEN];
		win->blob_xmax = blob_max[i];
		win->blob_ymax = blob_max[i + SEQ_LEN];

		// center camera's ROI 0 around the blob's midpoint in the image's coordinate frame.  
		// Note that in this implementation the initial placement of the ROI is dependent on 
		// the blob's initial coordinates.
		blob_cx = (win->blob_xmin + win->blob_xmax) / 2;
		blob_cy = (win->blob_ymin + win->blob_ymax) / 2;
		set_roi_box(win, blob_cx, blob_cy);

		// convert from the blob's image coordinate system to the ROI 
		// coordinate system.  This only needs to be done during initialization, 
		// because all routines in the tracking code assume that the blob is 
		// relative to the currently active ROI window and remain in that coordinate 
		// frame.
		fix_blob_bounds(win);

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
}

/** Draw ROI & blob windows and show image on screen (see OpenCV doc for info)
*
* display_tracking simply displays the current frame on screen.
*/

void display_tracking(TrackingWindow *cur, IplImage *gui, char *name)
{
	gui->imageData = (char *) cur->img;
	gui->imageDataOrigin = (char *) cur->img;

	// blob box
	cvRectangle(gui, cvPoint(cur->blob_xmin, 
		cur->blob_ymin), 
		cvPoint(cur->blob_xmax, cur->blob_ymax), 
		cvScalar(128));

	// show image
	cvShowImage(name, gui);

	// add a small delay, so OpenCV has time to display to screen
	cvWaitKey(1);
}

int cv_position(TrackingWindow *cur)
{
	int old_xoff, old_yoff, blob_cx, blob_cy;

	if(blob(cur) != OBJECT_FOUND) {
		return panic(cur);
	}

	old_xoff = cur->roi_xoff;
	old_yoff = cur->roi_yoff;

	// center roi around blob "center"
	blob_cx = old_xoff + (cur->blob_xmax + cur->blob_xmin) / 2;
	blob_cy = old_yoff + (cur->blob_ymax + cur->blob_ymin) / 2;
	set_roi_box(cur, blob_cx, blob_cy);
	
	// adjust coords of blob
	cur->blob_xmin -= (cur->roi_xoff - old_xoff);
	cur->blob_ymin -= (cur->roi_yoff - old_yoff);
	cur->blob_xmax -= (cur->roi_xoff - old_xoff);
	cur->blob_ymax -= (cur->roi_yoff - old_yoff);
	pad_blob_region(cur);

	return OBJECT_FOUND;
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
	int img_nr, cur_roi;
	TrackingSequence wins;
	TrackingWindow *cur;
	int seq[] = SEQ;
	unsigned int ts;
	__int64 start, stop, accum;
	CvMat *intrinsic, *distortion, *rot, *trans;
	CvMat *world, *distorted, *normalized;
	float z;

	assert(sizeof(unsigned int) == 4 && sizeof(float) == 4);

	// initialize packet...not good design should change
	packet[0] = 'R';
	packet[2] = 'X';
	packet[7] = 'Y';
	packet[12] = 'T';
	packet[PACKET_SIZE] = 'P';
	packet[PACKET_SIZE+2] = 'X';
	packet[PACKET_SIZE+7] = 'Y';
	packet[PACKET_SIZE+12] = 'T';

	// load camera model
	intrinsic = (CvMat *) cvLoad( "TrackCamIntrinsics062810.xml" );
	if(intrinsic == NULL) {
		return -EBADF;
	}

	distortion = (CvMat *) cvLoad( "TrackCamDistortion062810.xml" );
	if(distortion == NULL) {
		return -EBADF;
	}

	rot = (CvMat *) cvLoad( "TrackCamRotation062810.xml" );
	if(rot == NULL) {
		return -EBADF;
	}

	trans = (CvMat *) cvLoad( "TrackCamTranslation062810.xml" );
	if(rot == NULL) {
		return -EBADF;
	}

	world = cvCreateMat(3, 1, CV_32FC1);
	if(world == NULL) {
		return -ENOMEM;
	}

	normalized = cvCreateMat(1, 1, CV_32FC2);
	if(normalized == NULL) {
		return -ENOMEM;
	}

	distorted = cvCreateMat(1, 1, CV_32FC2);
	if(distorted == NULL) {
		return -ENOMEM;
	}

	// change priority class
	/*
	rc = SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS );
	if(rc == FALSE) {
      return GetLastError();
	} 

	rc = SetThreadPriority(GetCurrentThread(), HIGH_PRIORITY_CLASS);
	if(rc == FALSE) {
      return GetLastError();
   }
   */

	// initialize comm port
	rc = open_comm(PORT);
	if(rc != FG_OK) {
		printf("main: error opening comm port %s\n", PORT);
		return rc;
	}

	// following lines are for displaying images only!  See OpenCV doc for more info.
	// they can be left out, if speed is important.
	IplImage *cvDisplay[APP_NUM_ROI];
	IplImage *pyr;
	IplImage *canny;


	for(rc = 0; rc < APP_NUM_ROI; rc++) {
		cvDisplay[rc] = cvCreateImageHeader(cvSize(ROI_BOX, ROI_BOX), 
			BITS_PER_PIXEL, NUM_CHANNELS);
	}
	pyr = cvCreateImage(cvSize(cvDisplay[0]->width/2, cvDisplay[0]->height/2), 8, 1);
	canny = cvCreateImage(cvGetSize(cvDisplay[0]), 8, 1);

#if SHOW_DISP
	for(rc = 0; rc < APP_NUM_ROI; rc++) {
		cvNamedWindow(disp[rc], CV_WINDOW_AUTOSIZE);
		cvNamedWindow(disp[rc], CV_WINDOW_AUTOSIZE);
	}
#endif

	// initialize the tracking windows (i.e. blob and ROI positions)
	wins.seq = seq;
	wins.seq_len = SEQ_LEN;
	memset(&wins, 0, sizeof(TrackingSequence));
	set_initial_positions(wins.windows);

	// initialize the camera
	rc = init_cam(&fg, MEMSIZE(ROI_BOX, ROI_BOX), NUM_BUFFERS, CAMLINK);
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
	accum = 0;
	img_nr = 1;
	z = CV_MAT_ELEM(*trans, float, 2, 0);
	int old_img_nr = 0, send = 0;

	// start image loop and don't stop until the user presses 'q'
	printf("press 'q' at any time to quit this demo.\n");
	while(!(_kbhit() && _getch() == 'q')) {
		QueryPerformanceCounter((PLARGE_INTEGER) &start);
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		if(img_nr < FG_OK) {
			printf("loop getLast: %s\n", Fg_getLastErrorDescription(fg));
			break;
		}

#if !SHOW_DISP
		if(img_nr - old_img_nr > 1) {
			//printf("\nlost an image %d %d\n", img_nr, old_img_nr);
			//break;
		}
		old_img_nr++;
#endif
		
		// get image tag, tag == X => ROI_X
		cur_roi = img_nr;
		rc = Fg_getParameter(fg, FG_IMAGE_TAG, &cur_roi, PORT_A);
		if(rc != FG_OK) {
			printf("loop tag: %s\n", Fg_getLastErrorDescription(fg));
			break;
		}
		
		ts = img_nr;
		rc = Fg_getParameter(fg, FG_TIMESTAMP, &ts, PORT_A);
		if(rc != FG_OK) {
			printf("loop ts: %s\n", Fg_getLastErrorDescription(fg));
			break;
		}
		
		// get roi associated with image and point to image data
		cur_roi = cur_roi >> 16;
		cur = &(wins.windows[cur_roi]);
		cur->img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		// make sure that camera returned a valid image
		if(cur->img != NULL) {
			// process image...this takes about 82 us (eyeball estimation)
			cvDisplay[cur_roi]->imageData = (char *) cur->img;
			cvDisplay[cur_roi]->imageDataOrigin = (char *) cur->img;
			cvPyrDown(cvDisplay[cur_roi], pyr, CV_GAUSSIAN_5x5);
			cvPyrUp(pyr, cvDisplay[cur_roi], CV_GAUSSIAN_5x5);
			cvCanny(cvDisplay[cur_roi], cvDisplay[cur_roi], THRESHOLD0, THRESHOLD1);

			// update ROI position
			rc = cv_position(cur);
			if(rc == OBJECT_FOUND) {
				roi_packet[cur_roi].roi = cur_roi;
			}
			else {
				roi_packet[cur_roi].roi = ~cur_roi;
				printf("loop: object %d lost! too fast.\n", cur_roi);
#if !SHOW_DISP
				break;
#endif
			}

			if(cur->blob_xmax - cur->blob_xmin > ROI_BOX - 1 && 
				cur->blob_ymax - cur->blob_ymin > ROI_BOX - 1) {
				printf("loop: object %d lost! too much noise\n", cur_roi);
#if !SHOW_DISP
				break;
#endif
			}
			

			// at this point position(...) has updated the ROI, but it only stores
			// the updated values internal to the code.  The next step is to flush
			// the ROI to the camera (see position(...) documentation).

			// write ROI position to camera
			write_roi(fg, cur->roi, cur->roi, !DO_INIT);

			// convert from pixels to units of measurement
			
			distorted->data.fl[0] = cur->roi_xoff + 
				(cur->blob_xmin + cur->blob_xmax) / 2.0f;
			distorted->data.fl[1] = cur->roi_yoff + 
				(cur->blob_ymin + cur->blob_ymax) / 2.0f;
			cvUndistortPoints(distorted, normalized, intrinsic, distortion);

			CV_MAT_ELEM(*world, float, 0, 0) = z*normalized->data.fl[0];
			CV_MAT_ELEM(*world, float, 1, 0) = z*normalized->data.fl[1];
			CV_MAT_ELEM(*world, float, 2, 0) = z*1;
			cvSub(world, trans, world);
			cvGEMM(rot, world, 1, NULL, 0, world, CV_GEMM_A_T);
			
			roi_packet[cur_roi].x = CV_MAT_ELEM(*world, float, 0, 0);
			roi_packet[cur_roi].y = CV_MAT_ELEM(*world, float, 1, 0);
			roi_packet[cur_roi].t = ts;

			send++;
			if(cur_roi == ROI_1) {
				send = 0;
				for(rc = 0; rc < APP_NUM_ROI; rc++) {
					packet[rc*PACKET_SIZE + 1] = (char) roi_packet[rc].roi;
					memcpy(&packet[rc*PACKET_SIZE + 3], &roi_packet[rc].x, sizeof(float));
					memcpy(&packet[rc*PACKET_SIZE + 8], &roi_packet[rc].y, sizeof(float));
					memcpy(&packet[rc*PACKET_SIZE + 13], &roi_packet[rc].t, sizeof(unsigned int));
				}

				assert(packet[0] == 'R');
				assert(packet[2] == 'X');
				assert(packet[7] == 'Y');
				assert(packet[12] == 'T');
				assert(packet[PACKET_SIZE] == 'P');
				assert(packet[PACKET_SIZE+2] == 'X');
				assert(packet[PACKET_SIZE+7] == 'Y');
				assert(packet[PACKET_SIZE+12] == 'T');
		
				rc = write_comm(packet, APP_NUM_ROI*PACKET_SIZE);
				if(rc != FG_OK) {
					printf("loop comm: error writing to comm port %s\n", PORT);
					break;
				}
			}

			// show image on screen
#if SHOW_DISP
			display_tracking(cur, cvDisplay[cur_roi], disp[cur_roi]);
#endif

			// increment to the next desired frame.
			img_nr += NEXT_IMAGE;
		}
		else {
			// typically this state only occurs if an invalid ROI has been programmed
			// into the camera (e.g. roi_w == 4).
			printf("img %d is null\n", img_nr);
			break;
		}
		QueryPerformanceCounter((PLARGE_INTEGER) &stop);
		accum += (stop - start);
	}

	if(QueryPerformanceFrequency((PLARGE_INTEGER) &start) == TRUE) {
		printf("\naverage time %g s, total number of images: %d\n", accum / (img_nr * start * 1.0), img_nr);
	}

	// free viewer resources
	for(rc = 0; rc < APP_NUM_ROI; rc++) {
		cvReleaseImageHeader(&cvDisplay[rc]);
	}
	cvReleaseImage(&canny);
	cvReleaseImage(&pyr);

	// free camera resources
	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	close_comm();

	return FG_OK;
}