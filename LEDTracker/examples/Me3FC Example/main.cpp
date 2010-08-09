#include "t_dah.h"
#include "TDahMe3Fc.h"
#include <conio.h>

// me3 parametesr
#define TRIG GRABBER_CONTROLLED
#define EXPOSURE 50 // us
#define FRAME 100 // us
#define BUFS 16

// calib parameters
#define NONE 0
#define INTRINSIC 1
#define EXTRINSIC 2
#define CALIB 0

#define INTRINS_FILE "TrackCam Intrinsics.yaml"
#define EXTRINS_FILE "TrackCam Extrinsics.yaml"

#define GRID_W 3
#define GRID_H 6
#define INCHES_PER_SQR 1.33858268
#define NUM_IMGS 10

// ROI parameters
#define NUM_ROI 1
#define ROI_W 12
#define ROI_H 12

#define HAVE_CONF 0
#define CONF "myme3.yaml"
#define USE_KAL false
#define USE_TPT false

#define GR_CH 1

int track_dots(TDahMe3Fc *capture)
{
	int i;
	ROILoc r;

	if(HAVE_CONF) {
		// a config file exists, use it to auto-acquire dots
		i = capture->initROIs(NUM_ROI, CONF, USE_KAL, USE_TPT);
	}
	else {
		// explicitly give ROI info and save config for later use
		i = capture->initROIs(NUM_ROI, ROI_W, ROI_H, CONF, USE_KAL, USE_TPT,
			INTRINS_FILE, EXTRINS_FILE);
	}

	if(i != CV_OK) {
		return i;
	}

	i = 0;

	//cvNamedWindow("obj 0", 0);
	//cvNamedWindow("obj 1", 0);
	//cvNamedWindow("obj 2", 0);
	//cvNamedWindow("obj img", 0);
	//cvResizeWindow("obj img", ROI_W, ROI_H);

	//CvVideoWriter *writer = NULL;
	//writer = cvCreateVideoWriter("output.avi", CV_FOURCC('D', 'I', 'B', ' ') , 1e6 / FRAME,
	//						cvSize(FC_MAX_WIDTH,FC_MAX_HEIGHT), FALSE);

	//if(!writer) {
	//	return !CV_OK;
	//}

	r.img = cvCreateImage(cvSize(1024, 1024), 8, 1);
	while(1 || cvWaitKey(1) != 'q') {
		if(_kbhit()) break;

		// get and process next image
		capture->grabFrame();
		capture->getROILoc(++i, &r);
		//cvResetImageROI(r.img);
		//cvWriteFrame(writer, r.img);

		//cvShowImage("img", cvQueryFrame(capture));

		// visualize tracking
		//capture->showROILoc();
		//if(!r.obj_found) {
			//printf("%d (%d, %d) %d\n", r.roi_nr, r.loc.x, r.loc.y, r.obj_found);
			//IplImage *img = cvQueryFrame(capture);
			//cvShowImage("w", img);
		//}
	}
	//cvReleaseVideoWriter(&writer);
	
	return CV_OK;
}

int main()
{
	int rc;
	TDahMe3Fc *capture = new TDahMe3Fc(TRIG, EXPOSURE, FRAME, BUFS);

	if(CALIB == INTRINSIC) {
		rc = get_camera_intrinsics(capture, INTRINS_FILE, 
			GRID_W, GRID_H, NUM_IMGS);
	}
	else if(CALIB == EXTRINSIC) {
		rc = get_camera_extrinsics(capture, EXTRINS_FILE, INTRINS_FILE, 
			GRID_W, GRID_H, INCHES_PER_SQR);
	}
	else {
		rc = track_dots(capture);
	}

	delete capture;
	return rc;
}