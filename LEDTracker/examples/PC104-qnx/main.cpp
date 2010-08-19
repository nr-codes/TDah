#include "t_dah.h"
#include "TDahMe3Fc.h"

// me3 parametesr
#define TRIG ASYNC_TRIGGER
#define EXPOSURE 300 // us
#define FRAME EXPOSURE
#define BUFS 100

// calib parameters
#define NONE 0
#define INTRINSIC 1
#define EXTRINSIC 2
#define CALIB 0

#define CONF_FILE NULL
#define INTRINS_FILE "TrackCam_Intrins.yaml"
#define EXTRINS_FILE "TrackCam_Extrins.yaml"

#define GRID_W 3
#define GRID_H 6
#define MM_PER_SQR 30.1625
#define NUM_IMGS 16

// tracking-related functions
extern int output_analog(double x, double y, int roi);
extern int track_dots(TDahMe3Fc *capture, char *conf, char *ints, char *extr);

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
			GRID_W, GRID_H, MM_PER_SQR);
	}
	else {
		rc = track_dots(capture, CONF_FILE, INTRINS_FILE, EXTRINS_FILE);
	}
	
	delete capture;
	return rc;
}