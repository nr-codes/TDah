#include "t_dah.h"

static void onclick_center_rect(int e, int x, int y, int flags, void *param);
static IplImage *gr3chImg(IplImage *img, IplImage *gr);

static IplImage *tempImg = NULL;

//***************************** HELPER FUNCTIONS *********************//

static IplImage *gr3chImg(IplImage *img, IplImage *gr)
{
	if(tempImg) {
		cvReleaseImage(&tempImg);
	}

	if(img->nChannels == 3 && img->depth == IPL_DEPTH_8U) {
		tempImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvCvtColor(img, tempImg, CV_BGR2GRAY);
		cvCvtColor(tempImg, img, CV_GRAY2BGR);

		if(gr->roi) {
			cvSetImageROI(tempImg, cvGetImageROI(gr));
			cvCopyImage(tempImg, gr);
		}

		return img;
	}
	else if(img->nChannels == 1 && img->depth == IPL_DEPTH_8U) {
		tempImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
		cvCvtColor(img, tempImg, CV_GRAY2BGR);

		if(gr->roi) {
			cvSetImageROI(img, cvGetImageROI(gr));
			cvCopyImage(img, gr);
		}

		return tempImg;
	}

	return NULL;
}

void update_kal_tplt(IplImage *gr, CvKalman *kal, IplImage *tplt)
{
	static double dt_k = cvGetTickCount()/cvGetTickFrequency();
	CvPoint c;
	float z_k[Z_DIM];

	if(gr->roi == NULL) {
		return;
	}

	// update Kalman filter
	if(kal != NULL) {
		dt_k = (cvGetTickCount()/cvGetTickFrequency() - dt_k)*1e-6;
		c = roi2ctrd(gr);
		z_k[0] = (float) c.x;
		z_k[1] = (float) c.y;

		// beware a bad process model will cause a program crash
		prediction(kal, (float) dt_k, z_k);
		ctrd2roi(gr, 
			cvRound(kal->state_post->data.fl[0]), 
			cvRound(kal->state_post->data.fl[1]),
			ROI_WIDTH, 
			ROI_HEIGHT);
		dt_k = cvGetTickCount()/cvGetTickFrequency();
	}

	// update template
	if(tplt != NULL) {
		// note: if rect is out of image bounds OpenCV resizes ImageROI
		// to largest valid bounding box
		cvSetImageROI(tplt, cvGetImageROI(gr));
		cvCopyImage(gr, tplt);
	}
}

//***************************** AUTO CALIBRATION *********************//

int auto_acquire(CvCapture *capture, IplImage **gr, int t, double r, int n, 
				 IplImage **tplt, double m, CvKalman **kal, int show_results)
{
	double score;
	int found;
	int last_tplt;
	IplImage *img;
	CvSeqWriter wr;
	CvMemStorage *mem;

	mem = cvCreateMemStorage(0);
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);

	found = 0;
	score = 0;
	last_tplt = -1;
	img = gr3chImg(cvQueryFrame(capture), gr[0]);
	for(int i = 0; i < n; i++) {
		img = gr3chImg(img, gr[i]);
		if(gr[i]->roi) {
			score = track_ctrd(gr[i], t, &wr);
			printf("%d: centroid radius %0.4g\n", i, score);
			if(score == 0 || score > r) {
				// couldn't find dot, redo this img with tmplt matching
				cvResetImageROI(gr[i]);
				if(last_tplt == i) {
					// already tried template matching
					continue;
				}

				i--;
			}
			else {
				found++;
				update_kal_tplt(gr[i], 
					kal ? kal[i] : NULL, tplt ? tplt[i] : NULL);
			}
		}
		else if(tplt && tplt[i]) {
			cvCvtColor(img, gr[i], CV_BGR2GRAY);
			score = track_tmplt(gr[i], tplt[i]);
			printf("%d: template match %0.4g\n", i, score);
			if(score < m) {
				cvResetImageROI(gr[i]);
			}
			else {
				// use template as initial guess
				last_tplt = i;
				i--;
			}
		}
		else {
			printf("missing ROI or template for image %d\n", i);
		}
	}

	if(show_results) {
		for(int i = 0; i < n; i++) {
			if(gr[i]->roi) {
				if(kal && kal[i]) {
					draw_kal(img, kal[i]);
				}

				draw_ctrd(img, gr[i], NULL, i);
			}
		}
		cvShowImage("auto_acquire", img);
	}

	return !(found == n);
}

//***************************** MANUAL CALIBRATION *********************//

// shared variables between manual_acquire & mouse callback function
static int cur_roi;
CvPoint mouse_loc;

int manual_acquire(CvCapture *capture, IplImage **gr, int *t, int n, 
				   IplImage **tplt, CvKalman **kal)
{
	int rc;
	float z_k[Z_DIM];
	double dt_k;
	CvPoint c;
	CvFont font;
	CvSeqWriter wr;
	CvMemStorage *mem;
	IplImage *img;
	char text[100];
	
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	mem = cvCreateMemStorage(0);
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);

	cur_roi = 0;
	img = cvQueryFrame(capture);

	*t = THRESHOLD;
	cvNamedWindow("manual_acquire");
	cvCreateTrackbar("manual_acquire_track", "manual_acquire", t, WHITE, NULL);
	cvSetMouseCallback("manual_acquire", onclick_center_rect, gr);

	for(int j = 0; j < n; j++) {
		cvResetImageROI(gr[j]);
	}

	dt_k = cvGetTickCount()/cvGetTickFrequency();
	while(1) {
		img = gr3chImg(cvQueryFrame(capture), gr[cur_roi]);

		rc = cvWaitKey(100);
		if(rc == 'q') {
			break;
		}
		
		if(isdigit(rc) && (rc  - 0x30) < n) {
			cur_roi = rc - 0x30;
		}
		
		if(rc != ' ') {
			cvThreshold(img, 
				img, 
				cvGetTrackbarPos("manual_acquire_track", "manual_acquire"), 
				WHITE,
				CV_THRESH_BINARY_INV);
		}

		for(int j = 0; j < n; j++) {
			if(gr[j]->roi == NULL) {
				continue;
			}

			update_kal_tplt(gr[j], 
					kal ? kal[j] : NULL, 
					tplt ? tplt[j] : NULL);

			if(kal && kal[j]) {
				draw_kal(img, kal[j]);
			}

			draw_ctrd(img, gr[j], NULL, j);

#if 0
			c = roi2ctrd(gr[j]);
	
			// update Kalman filter
			if(kal != NULL && kal[j] != NULL) {
				z_k[0] = (float) c.x;
				z_k[1] = (float) c.y;
				dt_k = (cvGetTickCount()/cvGetTickFrequency() - dt_k)*1e-6;

				// beware a bad process model will cause a program crash
				prediction(kal[j], (float) dt_k, z_k);
				ctrd2roi(gr[j], 
					cvRound(kal[j]->state_post->data.fl[0]), 
					cvRound(kal[j]->state_post->data.fl[1]),
					ROI_WIDTH, 
					ROI_HEIGHT);

				draw_kal(img, kal[j]);
				dt_k = cvGetTickCount()/cvGetTickFrequency();
			}

			// update template
			if(tplt != NULL && tplt[j] != NULL) {
				// note: if rect is out of image bounds OpenCV resizes ImageROI
				// to largest valid bounding box
				cvSetImageROI(tplt[j], cvGetImageROI(gr[j]));
				cvCopyImage(gr[j], tplt[j]);
			}

			// draw the (x, y) point and bounding box
			draw_ctrd(img, gr[j], NULL, j);
#endif
		}

		memset(text, 0, sizeof(text));
		sprintf(text, "%d", cur_roi);
		cvPutText(img, 
			text, 
			mouse_loc,
			&font, 
			CV_RGB(0, 255, 0));

		cvShowImage("manual_acquire", img);
	}

	cvDestroyWindow("manual_acquire");
	return CV_OK;
}

void onclick_center_rect(int e, int x, int y, int flags, void *param)
{
	IplImage **gr;

	if(e == CV_EVENT_LBUTTONDOWN) {
		gr = (IplImage **) param;
		ctrd2roi(gr[cur_roi], x, y, ROI_WIDTH, ROI_HEIGHT);
	}
	else if(e == CV_EVENT_MOUSEMOVE) {
		mouse_loc.x = x;
		mouse_loc.y = y;
	}
}