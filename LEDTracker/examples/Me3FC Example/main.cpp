#include "t_dah.h"
#include "TDahMe3Fc.h"

int main()
{
	int i = 0;
	ROILoc r;
	IplImage *img;
	TDahMe3Fc *capture = new TDahMe3Fc(GRABBER_CONTROLLED, 20, 50, 16);

	img = cvQueryFrame(capture);
	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);

	capture->initROIs(1, 12, 12, "myme3.yaml", true, false);
	//capture->initROIs(1, "myme3.yaml", true, false);

	r.img = img;

	while(cvWaitKey(100) != 'q') {
		cvZero(img);
		capture->grabFrame();
	TIME_CODE("getROILOC",
		capture->getROILoc(++i, &r);
	);
		capture->showROILoc();

		//img = cvQueryFrame(capture);

		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	delete capture;
	return 0;
}