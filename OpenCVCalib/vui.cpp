#include <fcdynamic.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "grab.h"

#define MAIN "mainWin"
#define NUM_INPUTS 3
#define SOURCE WEBCAM

int main(int argc, char *argv[])
{
	int input;
	IplImage *img = NULL;
	char *ins, *dis;

	// any inputs?
	if(argc != NUM_INPUTS) {
		ins = "../Intrinsics.xml";
		dis = "../Distortion.xml";
	}
	else {
		ins = argv[0];
		dis = argv[1];
	}

	if(init_cam_src(SOURCE) != 0) {
		return -3;
	}
	img = grab(SOURCE);

	// create a window
	cvNamedWindow("Distorted", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Undistorted", CV_WINDOW_AUTOSIZE);

	// load camera matrices
	CvMat *intrinsic = (CvMat*)cvLoad( ins );
	CvMat *distortion = (CvMat*)cvLoad( dis );

	// undistortion code
	IplImage *mapx = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	IplImage *mapy = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

	while(1) {
		img = grab(SOURCE);

		if(img != NULL) {
			IplImage *t = cvCloneImage(img);
			cvShowImage("Distorted", img); // Show raw image
			cvRemap(t, img, mapx, mapy); // undistort image
			cvReleaseImage(&t);
			cvShowImage("Undistorted", img); // Show corrected image
		}
		else {
			printf("img is null\n");
			break;
		}

		// get input
		input = cvWaitKey(1);
		if(input == 'q') {
			break;
		}
	}

	close_cam_src(SOURCE);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);

	return 0;
}
