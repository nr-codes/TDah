#include "t_dah.h"
#include "TDahOpenCV.h"

int main()
{
	TDahOpenCV *capture;// = new TDahOpenCV(CV_CAP_ANY);
	ROILoc r;
	IplImage *img;
	float z[Z_DIM];


	CvKalman *kal = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
	setup_kalman(&kal);
	z[0] = 0; z[1] = 0;
	prediction(kal, 1, z);


	cvNamedWindow("f");
	cvWaitKey();
	cvReleaseKalman(&kal);
	return 0;


	//capture->initROIs(1, 20, 20, "myopencv.yaml", true, true);
	capture->initROIs(1, "myopencv.yaml", true, true);
	img = cvQueryFrame(capture);
	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	r.img = img;
	while(cvWaitKey(100) != 'q') {
		capture->showROILoc();
		capture->grabFrame();
		capture->getROILoc(3, &r);
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	delete capture;
	return 0;
}