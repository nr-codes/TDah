#include "t_dah.h"
#include "TDahOpenCV.h"

#define HAVE_CONF 0
#define CONF_FILE "myopencv.yaml"

// calib parameters
#define INTRINSIC 1
#define EXTRINSIC 2
#define CALIB 2

#define GRID_W 3
#define GRID_H 6
#define NUM_IMGS 75

#define INTRINS_FILE "Logitech Webcam Intrinsics.yaml"
#define EXTRINS_FILE "Logitech Webcam Extrinsics.yaml"

// ROI parameters
#define NUM_ROI 2
#define ROI_W 15
#define ROI_H 15

int track_dots(TDahOpenCV *capture)
{
	IplImage *img;
	ROILoc r;

	// setup ROIs
	if(HAVE_CONF && capture->initROIs(NUM_ROI, CONF_FILE, true, true, 
		INTRINS_FILE, EXTRINS_FILE) != CV_OK) {
		printf("couldn't initROIs\n");
		return !CV_OK;
	}
	else if(capture->initROIs(NUM_ROI, ROI_W, ROI_H, CONF_FILE, 
		true, true, INTRINS_FILE, EXTRINS_FILE) != CV_OK) {
		printf("couldn't initROIs\n");
		return !CV_OK;
	}

	img = cvQueryFrame(capture);
	if(img == NULL) {
		printf("couldn't get an image from capture device\n");
		return !CV_OK;
	}

	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	if(img == NULL) {
		printf("couldn't allocate new image\n");
		return !CV_OK;
	}

	r.img = img;
	while(cvWaitKey(100) != 'q') {
		// grab and process next image
		capture->grabFrame();
		capture->getROILoc(&r);

		// visualize tracking
		capture->showROILoc();
		cvShowImage("img", img);
		printf("%d (%0.4g, %0.4g)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	return CV_OK;
}

int main()
{
	int rc = CV_OK;
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);

	if(CALIB == INTRINSIC) {
		rc = get_camera_intrinsics(capture, INTRINS_FILE, 
			GRID_W, GRID_H, NUM_IMGS);
	}
	else if(CALIB == EXTRINSIC) {
		rc = get_camera_extrinsics(capture, EXTRINS_FILE, INTRINS_FILE, 
			GRID_W, GRID_H);
	}
	else {
		rc = track_dots(capture);
	}

	delete capture;
	return rc;
}