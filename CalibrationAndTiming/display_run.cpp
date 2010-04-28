/**
* @file display_run.cpp visually shows the tracking algorithm and allows the use to 
* manually set the initial bounding box around the object to be tracked.
*/

#include <fcdynamic.h>

#define MAIN_WIN "main"
#define THRESH_WIN "thresh"
#define THRESH_TRACK "thresh_track"

#define CHAR_TO_INT(c) ((c) - 0x30)

static void help()
{
	char hmsg[] = 
		"The GUI reacts to the following commands when a keyboard button is pressed:\n"
		"\n"
		"* Notation:\n"
		"* let 'x' represent the corresponding character on the keyboard\n"
		"* let '0-9' represent either the 0, 1, 2, 3,..., 9 buttons on the keyboard\n"
		"* let 'RMB' mean pressing the right mouse button\n"
		"* let 'HLMB' mean hold the left mouse button down\n"
		"\n"
		"* To relocate the i-th ROI, press\n"
		"*  'i': to enter relocate mode for the ROI\n"
		"*  '0-7': to select a particular ROI\n"
		"*  'RMB': to finally reposition the ROI\n"
		"\n"
		"* To set the object's bounding box, press:\n"
		"*  'HLMB': drag the mouse inside the ROI to draw the object's bounding box\n"
		"*  after drawing the desired box release the 'HLMB'\n"
		"\n"
		"* To threshold the image bounded by a ROI, press\n"
		"*  't': to enter threshold mode (make sure to set the object bounding box first!!!)\n"
		"*  adjust the slider to the desired value located at the top of the GUI\n"
		"\n"
		"* To track an object, press\n"
		"*  'p': to start tracking (make sure to set the ROI and object bounding box)\n"
		"\n"
		"* To step through the images in the GUI frame by frame, press:\n"
		"*  's': to enter step mode\n"
		"*  press any key, but 's' to advance to the next frame\n"
		"\n"
		"* To get this help message, press: \n"
		"*  'h': to print this help message \n"
		"\n"
		"* To quite the GUI, press: \n"
		"*  'q': to quit \n"
		"\n"
		"* press any key to continue";

		printf(hmsg);
		_getch();
}

/**
* shows the tracking software in action
*
* <code>display_run</code> shows a GUI with a live video feed from a camera or a collection
* of files from disk with the tracking algorithm running in the background.  The GUI
* displays a gray rectangle representing a ROI and a smaller gray rectange inside the ROI
* representing the blob's bounding box.  The user can set the desired threshold value and
* reposition the ROI at any time.  The GUI reacts to the following commands when a keyboard
* button is pressed:
*
* Notation:
* let 'x' represent the corresponding character on the keyboard
* let '0-9' represent either the 0, 1, 2, 3,..., 9 buttons on the keyboard
* let 'RMB' mean pressing the right mouse button
* let 'HLMB' mean hold the left mouse button down
*
* To relocate the i-th ROI, press
*	'i': to enter relocate mode for the ROI
*	'0-7': to select a particular ROI
*	'RMB': to finally reposition the ROI
*
* To set the object's bounding box, press:
*	'HLMB': drag the mouse inside the ROI to draw the object's bounding box
*	after drawing the desired box release the 'HLMB'
*
* To threshold the image bounded by a ROI, press
*	't': to enter threshold mode (make sure to set the object bounding box first!!!)
*	adjust the slider to the desired value located at the top of the GUI
*
* To track an object, press
*	'p': to start the tracking (make sure to position the ROI and object bounding box)
*
* To step through the images in the GUI frame by frame, press:
*	's': to enter step mode
*	press any key, but 's' to advance to the next frame
*
* To get help, press:
*	'h': to print a help message
*
* To quite the GUI, press:
*	'q': to quit
*
* @param tseq the TrackingSequence specifying the active ROIs and their initial positions
* in the image prior to tracking an object
* @param frame the frame time (e.g. length of time between images) in microseconds
* @param exposure the exposure time (e.g. length of time the shutter is kept open) in
* microseconds
*/

int display_run(TrackingSequence *tseq, double frame, double exposure)
{
	int rc, input = 0;
	int find_blob, do_thresh, calib, pause_frame;
	const int value = 0;
	int img_nr;
	int cur_win;
	TrackingWindow *cur;
	IplImage *cvDisplay = NULL;

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

	//initialize parameters
	rc = FG_OK;
	cur_win = 0;
	find_blob = 0;
	do_thresh = 0;
	img_nr = 1;
	calib = 0;
	pause_frame = 0;
	cur = tseq->windows + tseq->seq[cur_win];

	cvDisplay = cvCreateImage(cvSize(cur->img_w, cur->img_h), 8, 1);
	cvNamedWindow(MAIN_WIN, CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar(THRESH_TRACK, MAIN_WIN, (int *) &value, WHITE + 1, NULL);

#if ONLINE
	rc = StartGrabbing(&fg, tseq, NULL);
#else
	rc = StartGrabbing(NULL, tseq, &data);
#endif
	if(rc != FG_OK) {
		return rc;
	}

	// start image loop
	while(1) {
		cur = tseq->windows + tseq->seq[cur_win];
		cur_win++;
		cur_win %= tseq->seq_len;
#if ONLINE
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		cur->img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
#else
		GetNextImage(&faux_fg, img_nr, ANIMATION_NAME, ANIMATION_LENGTH, TRUE);
		cur->img = data;
		CopyImageToTrackingWindow(cur, faux_fg);
		img_nr++;
#endif

		if(cur->img != NULL) {
			// process image
			if(do_thresh) {
				threshold(cur, cvGetTrackbarPos(THRESH_TRACK, MAIN_WIN));
				erode(cur);
			}
			
			// copy image processing results
			CopyTrackingWindowToImage(cur, cvDisplay);

			// update roi
			if(find_blob) {
				rc = position(cur);
				if(rc != OBJECT_FOUND) {
					printf("blob lost in img %d!!!!  reinitialize tracker.\n", img_nr);
				}
			}

#if ONLINE
			write_roi(fg, cur->roi, img_nr + tseq->seq_len, !DO_INIT);
#endif

			// get input
			input = cvWaitKey(1);
			if(input == 'q') {
				break;
			}

			switch(input) {
				case 's':
					pause_frame = !pause_frame;
					break;
				case 'i':
					calib = !calib;
					if(!calib) {
						cvSetMouseCallback(MAIN_WIN, NULL, NULL);
					}
					break;
				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
					if(calib) {
						cvSetMouseCallback(MAIN_WIN, set_region, 
							tseq->windows + CHAR_TO_INT(input));
					}
					break;
				case 't':
					do_thresh = !do_thresh;
					break;
				case 'p':
					find_blob = !find_blob;
					break;
				case 'h':
					help();
					break;
			}

			// draw image
			if(calib) {
				// roi box
				cvRectangle(cvDisplay, 
							cvPoint(cur->roi_xoff, 
									cur->roi_yoff), 
							cvPoint(cur->roi_xoff + cur->roi_w, 
									cur->roi_yoff + cur->roi_h), 
							cvScalar(128));
			
				// blob box
				cvRectangle(cvDisplay, 
							cvPoint(cur->blob_xmin + cur->roi_xoff, 
									cur->blob_ymin + cur->roi_yoff), 
							cvPoint(cur->blob_xmax + cur->roi_xoff, 
									cur->blob_ymax + cur->roi_yoff), 
							cvScalar(128));
			}

			cvShowImage(MAIN_WIN, cvDisplay);

			printf("roi (%d): x %d y %d w %d h %d\n", cur->roi,
				cur->roi_xoff, cur->roi_yoff, cur->roi_w, cur->roi_h);

			printf("blob: x %d y %d w %d h %d\n",
				cur->blob_xmin, cur->blob_ymin, 
				cur->blob_xmax - cur->blob_xmin, cur->blob_ymax - cur->blob_ymin);

			if(pause_frame) {
				if(cvWaitKey(0) == 's') {
					pause_frame = 0;
				}
			}
		}
		else {
			printf("img is null: %d\n", img_nr);
			break;
		}
	}
	cvReleaseImage(&cvDisplay);

#if ONLINE
	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	if(cur->img == NULL) {
		return !FG_OK;
	}
#else
	cvReleaseImage(&faux_fg);
	free(data);
#endif

	return FG_OK;
}