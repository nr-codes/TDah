#include "t_dah.h"

#define APP_NUM_ROI 2
#define ROWS 2
#define COLS 1

int main()
{
	int rc;
	int t = 0, j;
	double score;
	CvCapture *capture;
	IplImage *tplt[APP_NUM_ROI], *img, *gr[APP_NUM_ROI];
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
	setup_kalman(kal, APP_NUM_ROI);

	// get dots
	rc = manual_acquire(capture, gr, &t, APP_NUM_ROI, tplt, kal);
	show_tplts(tplt, ROWS, COLS, APP_NUM_ROI);
	cvWaitKey(0);
	cvDestroyAllWindows();

	//rc = auto_acquire(capture, r, t, APP_NUM_ROI, tplt, kal);

	j = 0;
	start = cvGetTickCount()/cvGetTickFrequency();
	while(cvWaitKey(100) != 'q') {
		img = cvQueryFrame(capture);

		stop = cvGetTickCount()/cvGetTickFrequency();
		dt = (stop - start) * 1e-6;
		start = cvGetTickCount()/cvGetTickFrequency();

		cvSetImageROI(img, cvGetImageROI(gr[j]));
		cvCvtColor(img, gr[j], CV_BGR2GRAY);

		//score = track_ctrd(gr[j], t, &wr[j]);
		if(true || !score || score > 20) {
			cvResetImageROI(gr[j]);
			cvResetImageROI(img);
			cvCvtColor(img, gr[j], CV_BGR2GRAY);

			double score2 = track_tmplt(gr[j], tplt[j]);
			printf("x %d y %d w %d h %d (%g) -- ", 
				gr[j]->roi->xOffset, gr[j]->roi->yOffset, 
				gr[j]->roi->width, gr[j]->roi->height, score2);
			CvRect r = cvGetImageROI(gr[j]);

			cvResetImageROI(gr[j]);
TIME_CODE("tracking loop", 			
			score = track_tmplt_pyr(gr[j], tplt[j], 3);
); /* end timing */
			printf("x %d y %d w %d h %d (%g) -- ", 
				gr[j]->roi->xOffset, gr[j]->roi->yOffset, 
				gr[j]->roi->width, gr[j]->roi->height, score);


			printf("x %2.3g y %2.3g w %2.3g h %2.3g (%2.3g)\n", 
				gr[j]->roi->xOffset/(r.x*1.0), 
				gr[j]->roi->yOffset/(r.y*1.0), 
				gr[j]->roi->width/(r.width*1.0), 
				gr[j]->roi->height/(r.height*1.0), 
				score/score2);
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

		j++;
		j %= APP_NUM_ROI;
	}

	cvReleaseCapture(&capture);
	return 0;
}
