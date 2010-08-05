#include "t_dah.h"
#include "TDahOpenCV.h"

#define INTRINS_FILE "Logitech Webcam Intrinsics.yaml"
#define EXTRINS_FILE "Logitech Webcam Extrinsics.yaml"

int main()
{
	ROILoc r;
	IplImage *img;
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);

#if 0
#if 0
	if(capture->initROIs(2, 15, 15, "myopencv.yaml", true, true, 
		INTRINS_FILE, EXTRINS_FILE) != CV_OK) {
		printf("couldn't initROIs\n");
		return -1;
	}
#else
	if(capture->initROIs(2, "myopencv.yaml", true, true) != CV_OK) {
		printf("couldn't initROIs\n");
		return -1;
	}
#endif
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
		capture->showROILoc();
		capture->grabFrame();
		capture->getROILoc(&r);
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}
#else
	//get_camera_intrinsics(capture, INTRINS_FILE, 3, 6, 75);
	//get_camera_extrinsics(capture, EXTRINS_FILE, INTRINS_FILE, 3, 6, 1.34);
	
	img = cvQueryFrame(capture);

	CvMat *A, *k;
	CvFileStorage *fs = cvOpenFileStorage(INTRINS_FILE, NULL, CV_STORAGE_READ);
	read_intrinsic_params(fs, &A, &k);
	cvReleaseFileStorage(&fs);

	CvMat *R, *t;
	fs = cvOpenFileStorage(EXTRINS_FILE, NULL, CV_STORAGE_READ);
	read_extrinsic_params(fs, &R, &t);
	cvReleaseFileStorage(&fs);

	CvMat *pp = cvCreateMat(1,1, CV_32FC3);
	CvMat *wp = cvCreateMat(1,1, CV_32FC3);
	CvMat *Rvec = cvCreateMat(3, 1, CV_32FC1);

	IplImage *mapx = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	IplImage *mapy = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap(A, k, mapx, mapy);


	wp->data.fl[0] = 0;
	wp->data.fl[1] = 0;
	wp->data.fl[2] = 0;

	cvRodrigues2(R, Rvec);
	cvProjectPoints2(wp, Rvec, t, A, k, pp);
		printf("(%0.3f, %0.3f) <- %0.3f %0.3f %0.3f\n",
			pp->data.fl[0], 
			pp->data.fl[1], 
			wp->data.fl[0], 
			wp->data.fl[1], 
			wp->data.fl[2]);

	CvMat temp1, temp2;
	cvZero(k);
	cvProjectPoints2(wp, Rvec, t, A, k, pp);
	int ix = cvRound(pp->data.fl[0]);
	int iy = cvRound(pp->data.fl[1]);
	printf("(0,0) -> (%d, %d) -> (%g, %g)\n", ix, iy,
		cvmGet(cvGetMat(mapx, &temp1), ix, iy), 
		cvmGet(cvGetMat(mapy, &temp2), ix, iy));

	


	cvShowImage("temp", img);
	cvWaitKey(0);


#endif
	delete capture;
	return 0;
}
