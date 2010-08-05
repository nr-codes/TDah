#include "t_dah.h"
#include "TDahOpenCV.h"

#define HAVE_CONF 0

// calib parameters
#define INTRINSIC 1
#define EXTRINSIC 2
#define CALIB 0

#define GRID_W 3
#define GRID_H 6
#define NUM_IMGS 75

#define INTRINS_FILE "Logitech Webcam Intrinsics.yaml"
#define EXTRINS_FILE "Logitech Webcam Extrinsics.yaml"

// ROI parameters
#define NUM_ROI 2
#define ROI_W 15
#define ROI_H 15

int main()
{
	ROILoc r;
	IplImage *img;
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);

	if(CALIB == INTRINSIC) {
		return get_camera_intrinsics(capture, INTRINS_FILE, 
			GRID_W, GRID_H, NUM_IMGS);
	}
	else if(CALIB == EXTRINSIC) {
		return get_camera_extrinsics(capture, EXTRINS_FILE, INTRINS_FILE, 
			GRID_W, GRID_H);
	}

	// setup ROIs
	if(HAVE_CONF && capture->initROIs(NUM_ROI, "myopencv.yaml", true, true, 
		INTRINS_FILE, EXTRINS_FILE) != CV_OK) {
		printf("couldn't initROIs\n");
		return -1;
	}
	else if(capture->initROIs(NUM_ROI, ROI_W, ROI_H, "myopencv.yaml", 
		true, true, INTRINS_FILE, EXTRINS_FILE) != CV_OK) {
		printf("couldn't initROIs\n");
		return -1;
	}

	img = cvQueryFrame(capture);
	if(img == NULL) {
		printf("couldn't get an image from capture device\n");
		return -2;
	}

	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	if(img == NULL) {
		printf("couldn't allocate new image\n");
		return -3;
	}

	r.img = img;
	while(cvWaitKey(100) != 'q') {
		// grab and process next image
		capture->grabFrame();
		capture->getROILoc(&r);

		// visualize tracking
		capture->showROILoc();
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	delete capture;
	return 0;
}
