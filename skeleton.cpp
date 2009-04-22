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
#define EXPOSURE 200 /**< shutter speed in us */
#define FRAME_TIME 500 /**< pause between images in us (e.g. 1 / fps) */
#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define NUM_BUFFERS 16 /**< typical setting (max is 1,000,000 shouldn't exceed 1.6 GB)*/
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)

#define SEQ {ROI_0}
#define SEQ_LEN 1
#define CAMLINK FG_CL_DUALTAP_8_BIT

// CAMERA REGION OF INTEREST
#define ROI_BOX 128

// INITIAL BLOB POSITION IN IMG COORD FRAME
#define INITIAL_BLOB_XMIN (520)
#define INITIAL_BLOB_YMIN (550)
#define INITIAL_BLOB_WIDTH 50
#define INITIAL_BLOB_HEIGHT 50

// DURATION OF DATA COLLECTION IN SECONDS
#define DTIME 5

// APPLICATION-SPECIFIC PARAMETERS
#define BITS_PER_PIXEL 8
#define NUM_CHANNELS 1
#define THRESHOLD 80
#define DISPLAY "Simple Tracking" /**< name of display GUI */
#define NEXT_IMAGE 2 /**< next valid image to grab */

// PARAMETERS FOR COORDINATE TRANSFORMATION
    // INTRINSIC PARAMETERS
#define fc1 1266.4284821980687    // focal lengths
#define fc2 1267.0872016270605
#define cc1 525.4444073039037    // principle point
#define cc2 507.82196875937
#define kc1 -0.438355014235685    // distortion coefficients
#define kc2 0.383272634115685
#define kc3 -0.000173352516888
#define kc4 0.001387888614912
#define kc5 0.0000
#define alpha_c 0.0000    // skew coefficient
    // EXTRINSIC PARAMETERS
#define Rotx1 0.999998024892877
#define Rotx2 -0.001485493817206
#define Rotx3 0.001320423592483
#define Roty1 -0.001492242762369
#define Roty2 -0.999985753731942
#define Roty3 0.005124992165745
#define XCONV 200.1760775378343    // conversion factors, camera frame -> real world frame
#define YCONV 200.37829776822787
#define XTRANS 79.64661719630012    // camera frame -> real world frame translation
#define YTRANS 27.011541475687352

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
	win->roi_w = ROI_BOX;
	win->roi_h = ROI_BOX;
	win->img_w = IMG_WIDTH;
	win->img_h = IMG_HEIGHT;

	// store the camera's ROI 0 information
	SetTrackCamParameters(win + ROI_0, FRAME_TIME, EXPOSURE);

	// insert initial image coordinates of blob 0 (for software use)
	win->blob_xmin = INITIAL_BLOB_XMIN;
	win->blob_ymin = INITIAL_BLOB_YMIN;
	win->blob_xmax = INITIAL_BLOB_XMIN + INITIAL_BLOB_WIDTH;
	win->blob_ymax = INITIAL_BLOB_YMIN + INITIAL_BLOB_HEIGHT;

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

/** Draw ROI & blob windows and show image on screen (see OpenCV doc for info)
*
* display_tracking simply displays the current frame on screen.
*/

void display_tracking(TrackingWindow *cur, IplImage *gui)
{
	gui->imageData = (char *) cur->img;
	gui->imageDataOrigin = (char *) cur->img;

	// blob box
	cvRectangle(gui, cvPoint(cur->blob_xmin, 
		cur->blob_ymin), 
		cvPoint(cur->blob_xmax, cur->blob_ymax), 
		cvScalar(128));

	cvCircle(gui, cvPoint((cur->xcpix), (cur->ycpix)), 
		RADIUS, cvScalar(GRAY), THICKNESS);

	// show image
	cvShowImage(DISPLAY, gui);

	// add a small delay, so OpenCV has time to display to screen
	cvWaitKey(1);
}

/** Calculates the area centroid of the object of interest.
* prints out x,y coordinates of the centroid of the object.
*/

int centroid(TrackingWindow *win)
{
	int i, j;
	double m00, m10, m01;
	int roi_xmin, roi_ymin, roi_xmax, roi_ymax;
	int box_xmin, box_ymin, box_xmax, box_ymax;
	int p;

	m00 = m10 = m01 = 0;
	roi_xmin = win->blob_xmin;
	roi_ymin = win->blob_ymin;
	roi_xmax = win->blob_xmax;
	roi_ymax = win->blob_ymax;

	box_xmin = roi_xmax;
	box_ymin = roi_ymax;
	box_xmax = 0;
	box_ymax = 0;

	for(i = roi_ymin; i < roi_ymax; i++) {
		for(j = roi_xmin; j < roi_xmax; j++) {
			p = PIXEL(win, i, j);

			m00 += p; // area
			m10 += j * p; // xc * area
			m01 += i * p; //yc * area

			// find bounding rectangle
			if(p == FOREGROUND) {
				if(box_xmin > j) {
					box_xmin = j;
				}
				if(box_xmax < j) {
					box_xmax = j;
				}
				if(box_ymin > i) {
					box_ymin = i;
				}
				if(box_ymax < i) {
					box_ymax = i;
				}
			}
		}
	}

	win->A = m00;
	win->xc = m10 / m00;
	win->yc = m01 / m00;

	// if there is an object in view, print out
	// x,y coordinates of the centroid of the object
	//if (win->A) {
      // printf("%6.2f\t%6.2f\n%f\n", (win->xc + win->roi_xoff), (win->yc + win->roi_yoff), win->A);
	//}

	return !m00;
}

/** Transforms the x & y coordinates of the centroid from pixel coordinates in the camera frame 
*   to their normalized coordinates, using the same method as the 'normalize' function
*   in the Matlab calibration toolbox.
*   Then transforms coordinates from normalized coordinates to real world 
*   by multiplying by the focal length and adding a translation. 
*   Then divides by (pixels/mm) to convert from pixels to metric
*/
void trans_coords(TrackingWindow *win) {

	double x_p;    // pixel coordinates
	double y_p;
	double x_distort;
	double y_distort;
	double r_sq;    // position squared
	double k_radial;
	double delta_x;
	double delta_y;
	double x_n;   // normalized coordinates
	double y_n;
	double x_nrot;    // normalized coordinates with corrected rotation
	double y_nrot;
	double x_cent;
	double y_cent;

	// INTRINSIC TRANSFORMATIONS
	    // First: Subtract principal point, and divide by the focal length:
	x_p = (win->xc + win->roi_xoff);
	y_p = (win->yc + win->roi_yoff - 5);

	x_distort = (x_p - cc1)/fc1;
	y_distort = (y_p - cc2)/fc2;
        // Second: undo skew
    x_distort = x_distort - (alpha_c * y_distort);
	    // Third: Compensate for lens distortion:
	x_n = x_distort;
	y_n = y_distort;
	int kk;
	for (kk = 0; kk < 20; kk++) { 
	    r_sq = pow(x_n, 2) + pow(y_n, 2);
        k_radial =  1 + kc1 * r_sq + kc2 * pow(r_sq, 2) + kc5 * (r_sq, 3);
        delta_x = 2*kc3*x_n*y_n + kc4*(r_sq + 2*pow(x_n,2));
        delta_y = kc3 * (r_sq + 2*pow(y_n,2))+2*kc4*x_n*y_n;
        x_n = (x_distort - delta_x) / k_radial;
	    y_n = (y_distort - delta_y) / k_radial;
	}

	// EXTRINSIC TRANSFORMATIONS
	    // First: apply rotation compensation
	x_nrot = x_n*Rotx1 + y_n*Rotx2 + Rotx3;
	y_nrot = x_n*Roty1 + y_n*Roty2 + Roty3;

	    // Second: Undo normalization, apply translation
	x_cent = x_nrot*XCONV + XTRANS;
	y_cent = y_nrot*YCONV + YTRANS;

	win->xcpix = win->xc;
	win->ycpix = win->yc;
	win->xc = x_cent;
	win->yc = y_cent;


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

	int takedata = 0;  // takedata has value 0 or 1 depending on whether data is being recorded
	char key = '0';   // initialize command key
	int time;    // variable for recording time in microseconds	
	int inittime;    // initial time when data collection begins
	FILE *pF;    // pointer to text file
	pF = fopen("testdata.txt","w");    // open text file where data is to be stored
	int counter = 0;    // Counts the number of images taken

	char format[] = "Images/img%d.jpg";
    char filename[sizeof format+1000*DTIME];
	
	FrameInfo timing;

	// following lines are for displaying images only!  See OpenCV doc for more info.
	// they can be left out, if speed is important.
	IplImage *cvDisplay = NULL;

	cvDisplay = cvCreateImageHeader(cvSize(ROI_BOX, ROI_BOX), 
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
	int initnum;

	//QueryPerformanceFrequency(&timing.freq);

	// start image loop and don't stop until the user presses 'q'
	printf("press 'q' at any time to quit this demo.\n");
	printf("press 'd' to begin recording data.\n");
	while(!(key == 'q')) {

		//QueryPerformanceCounter(&timing.grab_start);
		
		// if a key is hit, assign it to variable 'key'
		if (_kbhit()) {
			key = _getch();
		}

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
			erode(&cur);
			
			// create timestamp
			time = img_nr;
			Fg_getParameter(fg, FG_TIMESTAMP, 
					&time, PORT_A);
			//printf("\n%d\n", time);
			
			// calculate centroid
			centroid(&cur);
			trans_coords(&cur);

			if (key == 'd') {
				printf("\nData collection in progress...\n");
				takedata = 1;
				inittime = time;
				initnum = img_nr;
				key = '0';
			}
			
			// Print time and coordinates of centroid to text file
			if (takedata) {
			    counter++;
				fprintf(pF, "%d\t%d\t\t%f\t%f\n", counter, (time-inittime), (cur.xc), 
				    (cur.yc));
				
				// Once the duration exceeds the desired length of the test, stop taking data
			    if (time-inittime > DTIME*1000000) {
				    takedata = 0;
				    printf("\nData collection complete.\n");
					
					// Save individual images to folder
					/*printf("\nSaving images...\n");
					int lastpic	= Fg_getLastPicNumber(fg, PORT_A) ;
					for (counter = initnum; counter <= lastpic; counter = counter + NEXT_IMAGE) {
                        sprintf(filename, format, counter + 1 - initnum);
						cvDisplay = cvCreateImage(cvSize(ROI_BOX, ROI_BOX), 
		                         BITS_PER_PIXEL, NUM_CHANNELS);
						cvDisplay->imageData = (char*) Fg_getImagePtr(fg, counter, PORT_A);
						cvSaveImage(filename, cvDisplay);
					}
					printf("\nImages saved.\n");*/
			    }
			}

			
			

			// update ROI position
			position(&cur);

			// at this point position(...) has updated the ROI, but it only stores
			// the updated values internal to the code.  The next step is to flushq
			// the ROI to the camera (see position(...) documentation).

			// write ROI position to camera to be updated on frame "img_nr"
			write_roi(fg, cur.roi, img_nr, !DO_INIT);

			// show image on screen
			//display_tracking(&cur, cvDisplay);

			
		}
		else {
			// typically this state only occurs if an invalid ROI has been programmed
			// into the camera (e.g. roi_w == 4).
			printf("img is null: %d\n", img_nr);
			break;
		}	
			/*QueryPerformanceCounter(&timing.grab_stop);
			printf("Time elapsed: %lld\n", 1000000*(timing.grab_stop.QuadPart - 
				timing.grab_start.QuadPart) / timing.freq.QuadPart);
			printf("%d\n", img_nr);*/
	}
	
	fclose(pF);

	/*

	// Create video, add each image to video, write video to file

	printf("Creating video...\n");
	typedef struct CvVideoWriter CvVideoWriter;
	CvVideoWriter* writer;
	IplImage* ptrImg;
	char formatvid[] = "2.21.09-100HzThresh/img%d.jpg";
    char filenamevid[sizeof format+1000*DTIME];
    //writer = cvCreateVideoWriter( "100HzVid2.avi", -1, 10, cvSize(ROI_BOX, ROI_BOX), 0);
	for (counter = 1; counter <= 200; counter = counter + NEXT_IMAGE) {
		sprintf(filenamevid, formatvid, counter);
	    ptrImg = cvLoadImage(filenamevid, CV_LOAD_IMAGE_GRAYSCALE);
        cvThreshold(ptrImg, ptrImg, 128,
                  255, CV_THRESH_BINARY);
		cvShowImage(DISPLAY, ptrImg);
		cvWaitKey(1);
		//cvWriteFrame( writer, ptrImg );
	}
	//cvReleaseVideoWriter( &writer );

	// free viewer resources
	cvReleaseImageHeader(&cvDisplay);

	*/


	/* IplImage* ptrImg;
	char formatvid[] = "image.jpg";
    char filenamevid[sizeof format+1000*DTIME];
		sprintf(filenamevid, formatvid, counter);
	    ptrImg = cvLoadImage(filenamevid, CV_LOAD_IMAGE_GRAYSCALE);

        cur.img = (unsigned char *) ptrImg;
	    cur.roi_xoff = 0;
	    cur.roi_yoff = 0;
	    cur.roi_w = 1024;
	    cur.roi_h = 1024;
	
	    cur.blob_xmin = 0;
	    cur.blob_ymin = 0;
	    cur.blob_xmax = 1024;
        cur.blob_ymax = 1024;
	
	    cur.img_w = 1024;
	    cur.img_h = 1024;
		threshold(&cur, THRESHOLD);
		centroid(&cur);

	    printf("%d\n", cur.A);
	    cvWaitKey(1000); */

	// free camera resources
	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	return FG_OK;
}