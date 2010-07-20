#include "t_dah.h"

#define APP_NUM_ROI 2
#define MAX_RADIUS 10
#define MIN_MATCH 0.89
#define ROWS 2
#define COLS 1

int main()
{
	int rc;
	int state;
	int t = 0, j;
	double score;
	CvCapture *capture;
	IplImage *tplt[APP_NUM_ROI], *img, *gr[APP_NUM_ROI], *temp;
	CvKalman *kal[APP_NUM_ROI];
	CvSeqWriter wr[APP_NUM_ROI];
	CvMemStorage *mem[APP_NUM_ROI];
	CvPoint c;
	float z[Z_DIM];
	double dt, start, stop;

	// init everything
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	img = cvQueryFrame(capture);
	for(int i = 0; i < APP_NUM_ROI; i++) {
		gr[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		tplt[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvSetImageROI(gr[i], cvRect(0, 0, img->width, img->height));
		cvSetImageROI(tplt[i], cvRect(0, 0, img->width, img->height));

		kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
		mem[i] = cvCreateMemStorage(0);
		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
							sizeof(CvSeq), sizeof(CvPoint), mem[i], &wr[i]);
	}
	temp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	cvZero(temp);

	setup_kalman(kal, APP_NUM_ROI);

	// get dots
	rc = manual_acquire(capture, gr, &t, APP_NUM_ROI, tplt, kal);
	show_tplts(tplt, ROWS, COLS, APP_NUM_ROI);
	cvWaitKey(0);
	cvDestroyAllWindows();

	rc = auto_acquire(capture, gr, t, MAX_RADIUS, 
		APP_NUM_ROI, tplt, MIN_MATCH, kal, 1);


	cvNamedWindow("pyr", 0);
	j = 0;
	start = cvGetTickCount()/cvGetTickFrequency();
	while(cvWaitKey(100) != 'q') {
		img = cvQueryFrame(capture);

		stop = cvGetTickCount()/cvGetTickFrequency();
		dt = (stop - start) * 1e-6;
		start = cvGetTickCount()/cvGetTickFrequency();

		cvSetImageROI(img, cvGetImageROI(gr[j]));
		cvCvtColor(img, gr[j], CV_BGR2GRAY);

		state = 0;
		score = track_ctrd(gr[j], t, &wr[j]);
		if(!score || score > MAX_RADIUS) {
			cvResetImageROI(gr[j]);
			cvResetImageROI(img);
			cvCvtColor(img, gr[j], CV_BGR2GRAY);
			state = 1;
			score = track_tmplt_pyr(gr[j], tplt[j], temp, 2, 5);

			if(score < MIN_MATCH) {
				cvResetImageROI(gr[j]);
				state = 2;
				score = track_tmplt(gr[j], tplt[j]);
				if(score < MIN_MATCH) {
					state = 3;
				}
			}
		}

		c = roi2ctrd(gr[j]);
		z[0] = (float) c.x;
		z[1] = (float) c.y;
		prediction(kal[j], (float) dt, z);
		ctrd2roi(gr[j], 
					cvRound(kal[j]->state_post->data.fl[0]), 
					cvRound(kal[j]->state_post->data.fl[1]),
					ROI_WIDTH, 
					ROI_HEIGHT);

		cvResetImageROI(img);
		show_position(gr, APP_NUM_ROI, kal, wr, NULL, img);
		show_seqs(wr, ROWS, COLS, APP_NUM_ROI);

		cvResetImageROI(temp);
		cvShowImage("pyr", temp);
/*
		if(state == 0) {
			printf("%d: centroid -- %0.4g\n", j, score);
		}
		else if(state == 1) {
			printf("%d: pyramid template -- %0.4g\n", j, score);
		}
		else if (state == 2) {
			printf("%d: full template -- %0.4g\n", j, score);
		}
		else if(state == 3) {
			printf("%d: object lost -- %0.4g\n", j, score);
		}
*/

		j++;
		j %= APP_NUM_ROI;
	}

	cvReleaseCapture(&capture);
	return 0;
}
