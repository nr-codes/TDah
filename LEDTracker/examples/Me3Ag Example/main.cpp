#include "t_dah.h"
#include "TDahMe3Ag.h"

// me3 parametesr
#define TRIG GRABBER_CONTROLLED
#define EXPOSURE 20000 // us
#define FRAME 50000 // us
#define BUFS 100

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
#define NUM_IMGS 16

// ROI parameters
#define NUM_ROI 3
#define ROI_W 20
#define ROI_H 20

#define HAVE_CONF 0
#define CONF "myme3.yaml"
#define USE_KAL false
#define USE_TPT false

#define VIDEO_FILE "me3ag.avi"

#define GR_CH 1

int track_dots(TDahMe3Ag *capture)
{
	int i;
	ROILoc r;

	if(HAVE_CONF) {
		// a config file exists, use it to auto-acquire dots
		i = capture->initROIs(NUM_ROI, CONF, USE_KAL, USE_TPT);
	}
	else {
		// explicitly give ROI info and save config for later use
		i = capture->initROIs(NUM_ROI, ROI_W, ROI_H, CONF, USE_KAL, USE_TPT);
			//INTRINS_FILE, EXTRINS_FILE); DELETE
	}

	if(i != CV_OK) {
		return i;
	}

	i = 0;

	r.img = cvCreateImage(cvSize(AG_MAX_WIDTH, AG_MAX_HEIGHT), 8, 1);
	if(!r.img) return !CV_OK;
	while(cvWaitKey(100) != 'q') {
		// get and process next image
		capture->grabFrame();
		++i;
		capture->getROILoc(i, &r);

		// visualize tracking
		capture->showROILoc();
	}

	if(!capture->saveMe3Buffer(VIDEO_FILE)) {
		return !CV_OK;
	}
	
	return CV_OK;
}

int main()
{
	int rc;
	TDahMe3Ag *capture = new TDahMe3Ag();

	rc = !CV_OK; // assume failure
	if(capture->open(TRIG, EXPOSURE, FRAME, BUFS)) {
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
	}

	delete capture;
	return rc;
}