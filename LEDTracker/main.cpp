#include "t_dah.h"

#define APP_NUM_ROI 2
#define ROWS 2
#define COLS 1


int main()
{
	int rc;
	CvRect r[APP_NUM_ROI] = {0};
	int t = 0, j;
	CvCapture *capture;
	IplImage *tplt[APP_NUM_ROI], *img, *gr[APP_NUM_ROI];
	CvKalman *kal[APP_NUM_ROI];
	CvSeqWriter wr[APP_NUM_ROI];
	CvMemStorage *mem[APP_NUM_ROI];
	char text[100];
	float z[Z_DIM], dt;
	LARGE_INTEGER start, stop, freq;

	capture = cvCaptureFromCAM(CV_CAP_ANY);
	img = cvQueryFrame(capture);
	for(int i = 0; i < APP_NUM_ROI; i++) {
		gr[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		tplt[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
		mem[i] = cvCreateMemStorage(0);
		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
							sizeof(CvSeq), sizeof(CvPoint), mem[i], &wr[i]);
	}
	setup_kalman(kal, APP_NUM_ROI);

	// get dots
	rc = manual_acquire(capture, r, &t, APP_NUM_ROI, tplt, kal);
	show_tplts(tplt, ROWS, COLS, APP_NUM_ROI);
	cvWaitKey(0);
	cvDestroyAllWindows();

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

		cvSetImageROI(gr[j], r[j]);
		cvSetImageROI(img, r[j]);
		cvCvtColor(img, gr[j], CV_BGR2GRAY);

		if(position(gr[j], &r[j], t, &wr[j]) != CV_OK) {
			cvResetImageROI(gr[j]);
			cvResetImageROI(img);
			cvCvtColor(img, gr[j], CV_BGR2GRAY);
			emergency(&r[j], gr[j], tplt[j]);
		}

		z[0] = (float) r[j].x + r[j].width/2;
		z[1] = (float) r[j].y + r[j].height/2;
		prediction(kal[j], dt, z);

		// update ROI
		r[j].x = cvRound(
			kal[j]->state_post->data.fl[0] - r[j].width/2);
		r[j].y = cvRound(
			kal[j]->state_post->data.fl[1] - r[j].height/2);
		cvSetImageROI(gr[j], r[j]);

		cvResetImageROI(img);
		show_position(gr, APP_NUM_ROI, kal, wr, NULL, img);
		show_seqs(wr, ROWS, COLS, APP_NUM_ROI);

		cvSetImageROI(img, r[j]);

		j++;
		j %= APP_NUM_ROI;
	}

	cvReleaseCapture(&capture);
	return 0;
}
