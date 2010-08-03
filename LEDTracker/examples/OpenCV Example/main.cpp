#include "t_dah.h"
#include "TDahOpenCV.h"

int main()
{
	ROILoc r;
	IplImage *img;
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);

	//get_camera_intrinsics(capture, 3, 6, 1);
	get_camera_extrinsics(capture, 3, 6, cvPoint2D32f(0,0), 0);
#if 0
	capture->initROIs(1, 30, 30, "myopencv.yaml", true, true);
	//capture->initROIs(1, "myopencv.yaml", true, false);
	img = cvQueryFrame(capture);
	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	r.img = img;
	while(cvWaitKey(100) != 'q') {
		capture->showROILoc();
		capture->grabFrame();
		capture->getROILoc(&r);
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}
#endif
	delete capture;
	return 0;
}
