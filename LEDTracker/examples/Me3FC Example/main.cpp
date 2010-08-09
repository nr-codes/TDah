#include "t_dah.h"
#include "TDahMe3Fc.h"

// me3 parametesr
#define TRIG GRABBER_CONTROLLED
#define EXPOSURE 20000 // us
#define FRAME 50000 // us
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
#define NUM_ROI 3
#define ROI_W 15
#define ROI_H 15

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
	while(cvWaitKey(100) != 'q') {
		// get and process next image
		capture->grabFrame();
		capture->getROILoc(++i, &r);

		// visualize tracking
		capture->showROILoc();
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

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