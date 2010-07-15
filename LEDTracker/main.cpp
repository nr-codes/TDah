#include "t_dah.h"

#define APP_NUM_ROI 2

int main()
{
	int rc;
	CvRect r[APP_NUM_ROI] = {0};
	int t = 0, j;
	CvCapture *capture;
	IplImage *tplt[APP_NUM_ROI] = {0}, *img, *gr;
	CvKalman *kal[APP_NUM_ROI] = {0};
	CvSeqWriter wr;
	CvMemStorage *mem;
	char text[100];
	float z[Z_DIM], dt;
	LARGE_INTEGER start, stop, freq;

	capture = cvCaptureFromCAM(CV_CAP_ANY);

	gr = cvCreateImage(cvGetSize(cvQueryFrame(capture)), IPL_DEPTH_8U, 1);
	for(int i = 0; i < APP_NUM_ROI; i++) {
		tplt[i] = cvCreateImage(cvGetSize(cvQueryFrame(capture)), 
					IPL_DEPTH_8U, 1);
		
		kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
	}

	mem = cvCreateMemStorage(0);
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);
	setup_kalman(kal, APP_NUM_ROI);


	// get dots
	rc = manual_acquire(capture, r, &t, APP_NUM_ROI, tplt, kal);

	for(int i = 0; i < APP_NUM_ROI; i++) {
		sprintf(text, "template %d", i);
		cvShowImage(text, tplt[i]);
	}
	cvWaitKey(0);


	//rc = auto_acquire(capture, r, t, APP_NUM_ROI, tplt, kal);

	for(int i = 0; i < APP_NUM_ROI; i++) {
		sprintf(text, "template %d", i);
		cvDestroyWindow(text);
	}

	j = 0;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	while(cvWaitKey(100) != 'q') {
		img = cvQueryFrame(capture);

		QueryPerformanceCounter(&stop);
		dt = (float) ((stop.QuadPart - start.QuadPart) / 
			((double) freq.QuadPart));
		QueryPerformanceCounter(&start);

		cvSetImageROI(gr, r[j]);
		cvSetImageROI(img, r[j]);
		cvCvtColor(img, gr, CV_BGR2GRAY);

		if(position(gr, &r[j], t, &wr) != CV_OK) {
			cvResetImageROI(gr);
			cvResetImageROI(img);
			cvCvtColor(img, gr, CV_BGR2GRAY);
			emergency(&r[j], gr, tplt[j]);
		}

		z[0] = (float) r[j].x + r[j].width/2;
		z[1] = (float) r[j].y + r[j].height/2;
		prediction(kal[j], dt, z);

		// update ROI
		r[j].x = cvRound(
			kal[j]->state_post->data.fl[0] - r[j].width/2);
		r[j].y = cvRound(
			kal[j]->state_post->data.fl[1] - r[j].height/2);
		cvSetImageROI(gr, r[j]);
		cvSetImageROI(img, r[j]);
		draw_position(gr, img, wr.seq, kal[j]);


		j++;
		j %= APP_NUM_ROI;
	}

	for(int i = 0; i < APP_NUM_ROI; i++) {
		cvReleaseImage(&tplt[i]);
		cvReleaseKalman(&kal[i]);
	}
	cvReleaseCapture(&capture);

	return 0;
}