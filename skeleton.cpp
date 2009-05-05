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
#include "constants.h"

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
	TrackingWindow cur, cur2;
	int seq[] = SEQ;

	int takedata = 0;  // takedata has value 0 or 1 depending on whether data is being recorded
	char key = '0';   // initialize command key
	int time;    // variable for recording time in microseconds	
	int inittime;    // initial time when data collection begins
	FILE *pF;    // pointer to text file
	pF = fopen(DESTINATION,"w");    // open text file where data is to be stored
	int counter = 0;    // Counts the number of images taken

	char format[] = "Images/img%d.jpg";
    //char filename[sizeof format+1000*DTIME];
	
	//FrameInfo timing;

	// following lines are for displaying images only!  See OpenCV doc for more info.
	// they can be left out, if speed is important.
	IplImage *cvDisplay = NULL;
	cvDisplay = cvCreateImageHeader(cvSize(ROI_BOX, ROI_BOX), 
		BITS_PER_PIXEL, NUM_CHANNELS);
	cvNamedWindow(DISPLAY, CV_WINDOW_AUTOSIZE);

	IplImage *cvDisplay2 = NULL;
    cvDisplay2 = cvCreateImageHeader(cvSize(ROI_BOX2, ROI_BOX2), 
		BITS_PER_PIXEL, NUM_CHANNELS);
	cvNamedWindow("Simple Tracking 2", CV_WINDOW_AUTOSIZE);
	
	// initialize the tracking window (i.e. blob and ROI positions)
	memset(&cur, 0, sizeof(TrackingWindow));
	set_initial_positions(&cur);

	memset(&cur2, 0, sizeof(TrackingWindow));
	set_initial_positions2(&cur2);

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

		if (img_nr%2 == 1) {
		    cur.img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
		}
		else {
			cur2.img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);
		}

		// make sure that camera returned a valid image

			if (img_nr%2 == 1) {
				if (cur.img == NULL) {
					// typically this state only occurs if an invalid ROI has been programmed
					// into the camera (e.g. roi_w == 4).
					printf("img is null: %d\n", img_nr);
					system("PAUSE");
					break;
				}
		        // process image
		        threshold(&cur, THRESHOLD);
		        erode(&cur);
		        // calculate centroid
		        centroid(&cur);
		        trans_coords(&cur);

			}

			else {
				if (cur2.img == NULL) {
					// typically this state only occurs if an invalid ROI has been programmed
					// into the camera (e.g. roi_w == 4).
					printf("img is null: %d\n", img_nr);
					system("PAUSE");
					break;
				}
				// process image
				threshold(&cur2, THRESHOLD);
				erode(&cur2);
			
				// calculate centroid
				centroid(&cur2);
				trans_coords(&cur2);
			}

			// create timestamp
			time = img_nr;
			Fg_getParameter(fg, FG_TIMESTAMP, 
					&time, PORT_A);
			//printf("\n%d\n", time);

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
				if (counter %2 == 0){
					fprintf(pF, "%d\t%d\t\t%f\t%f\t%f\t%f\n", counter/2, (time-inittime), (cur.xc), 
						(cur.yc), (cur2.xc), (cur2.yc));
				}

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

			if (img_nr%2 == 1) {
			// update ROI position
			position(&cur);

			// at this point position(...) has updated the ROI, but it only stores
			// the updated values internal to the code.  The next step is to flushq
			// the ROI to the camera (see position(...) documentation).

			// write ROI position to camera to be updated on frame "img_nr"
			write_roi(fg, cur.roi, img_nr + 4, !DO_INIT);

			// show image on screen
			//display_tracking(&cur, cvDisplay);
			}

			else {
			// update ROI position
			position(&cur2);

			// at this point position(...) has updated the ROI, but it only stores
			// the updated values internal to the code.  The next step is to flushq
			// the ROI to the camera (see position(...) documentation).

			// write ROI position to camera to be updated on frame "img_nr"
			write_roi(fg, cur2.roi, img_nr + 4, !DO_INIT);
			// show image on screen
			//display_tracking2(&cur2, cvDisplay2);
			}
			
			// increment to the next desired frame.  This has to be at least
			// +2, because the camera's ROI will not be active until the second
			// frame (see Silicon Software FastConfig doc)
			img_nr += NEXT_IMAGE;
			
			/*QueryPerformanceCounter(&timing.grab_stop);
			printf("Time elapsed: %lld\n", 1000000*(timing.grab_stop.QuadPart - 
				timing.grab_start.QuadPart) / timing.freq.QuadPart);
			printf("%d\n", img_nr);*/
	}
	
	fclose(pF);

	// free camera resources
	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}

	return FG_OK;
}