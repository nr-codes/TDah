#include "t_dah.h"

static void onclick_center_rect(int e, int x, int y, int flags, void *param);
static IplImage *gr3chImg(IplImage *img, IplImage *gr);

static IplImage *tempImg = NULL;

//***************************** HELPER FUNCTIONS *********************//

IplImage *gr3chImg(IplImage *img, IplImage *gr)
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

//***************************** AUTO CALIBRATION *********************//

int auto_acquire(CvCapture *capture, CvRect *r, int t, int n, 
				 IplImage **tplt, CvKalman **kal)
{
	int i;
	IplImage *img, *gr;
	CvRect rect;

	i = 10;
	while(i--) {
		// get rid of any transient image startups
		img = cvQueryFrame(capture);
	}

	gr = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	cvCvtColor(img, gr, CV_BGR2GRAY);

	// try template matching
	rect = cvRect(0, 0, ROI_WIDTH, ROI_HEIGHT);
	if(tplt != NULL) {
		for(i = 0; i < n; i++) {
			track_tmplt(gr, tplt[i]);
			r[i] = rect;
			cvResetImageROI(gr);

			int x = r[i].x + r[i].width/2;
			int y = r[i].y + r[i].height/2;
			cvCircle(gr, cvPoint(x, y), 2, CV_RGB(0,0, 255), -1);

			cvShowImage("gr", gr);
		}
	}

	return CV_OK;
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