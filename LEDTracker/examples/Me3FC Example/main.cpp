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
	r.img = img;

	capture->initROIs(1, 12, 12, "myme3.yaml", false, false);
	//capture->initROIs(1, "myme3.yaml", true, false);

	while(cvWaitKey(100) != 'q') {
		capture->grabFrame();
		capture->getROILoc(++i, &r);
		capture->showROILoc();
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	delete capture;
	return 0;
}