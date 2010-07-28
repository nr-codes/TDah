#include "t_dah.h"

int main()
{
	ROILoc r;
	IplImage *img;
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);

	//capture->initROIs(1, 15, 15, "myopencv.yaml", true, false);
	capture->initROIs(1, "myopencv.yaml", true, false);
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

	delete capture;
	return 0;
}