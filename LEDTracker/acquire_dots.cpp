#include "t_dah.h"

static void onclick_center_rect(int e, int x, int y, int flags, void *param);
static IplImage *gr3chImg(IplImage *img, IplImage *gr);

static IplImage *tempImg = NULL;

//***************************** HELPER FUNCTIONS *********************//

IplImage *gr3chImg(IplImage *img, IplImage *gr)
{
	if(img->nChannels == 3 && img->depth == IPL_DEPTH_8U) {
		cvCvtColor(img, gr, CV_BGR2GRAY);
		cvCvtColor(gr, img, CV_GRAY2BGR);
		return img;
	}
	else if(img->nChannels == 1 && img->depth == IPL_DEPTH_8U) {
		cvCopyImage(img, gr);

		if(tempImg) {
			cvReleaseImage(&tempImg);
		}

		tempImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
		cvCvtColor(img, tempImg, CV_GRAY2BGR);
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
			emergency(&rect, gr, tplt[i]);
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
static IplImage *img;
CvPoint mouse_loc;
static char text[100];

int manual_acquire(CvCapture *capture, CvRect *r, int *t, int n, 
				   IplImage **tplt, CvKalman **kal)
{
	int rc;
	int val, x_c, y_c;
	float z_k[Z_DIM], dt_k;
	IplImage *gr;
	CvFont font;
	CvSeqWriter wr;
	CvMemStorage *mem;
	LARGE_INTEGER start, stop, freq;
	
	QueryPerformanceFrequency(&freq);
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	mem = cvCreateMemStorage(0);
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);

	cur_roi = 0;
	img = cvQueryFrame(capture);
	gr = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);

	*t = THRESHOLD;
	cvNamedWindow("manual_acquire");
	cvCreateTrackbar("manual_acquire_track", "manual_acquire", t, WHITE, NULL);
	cvSetMouseCallback("manual_acquire", onclick_center_rect, r);

	for(int j = 0; j < n; j++) {
		r[j] = cvRect(BAD_ROI, BAD_ROI, ROI_WIDTH, ROI_HEIGHT);
	}

	QueryPerformanceCounter(&start);
	while(1) {
		img = gr3chImg(cvQueryFrame(capture), gr);
		QueryPerformanceCounter(&stop);

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
			if(r[j].x == BAD_ROI) {
				continue;
			}

			x_c = r[j].x + r[j].width/2;
			y_c = r[j].y + r[j].height/2;

			// update Kalman filter
			if(kal != NULL && kal[j] != NULL) {
				z_k[0] = (float) x_c;
				z_k[1] = (float) y_c;
				dt_k = (float) ((stop.QuadPart - start.QuadPart) / 
					((double) freq.QuadPart));

				// beware a bad process model will cause a program crash
				prediction(kal[j], dt_k, z_k);

				r[j].x = cvRound(
					kal[j]->state_post->data.fl[0] - r[j].width/2);
				r[j].y = cvRound(
					kal[j]->state_post->data.fl[1] - r[j].height/2);

				QueryPerformanceCounter(&start);
			}

			// update template
			if(tplt != NULL && tplt[j] != NULL) {
				// note: if rect is out of image bounds OpenCV resizes ImageROI
				// to largest valid bounding box
				cvSetImageROI(gr, r[j]);
				cvSetImageROI(tplt[j], r[j]);
				cvCopyImage(gr, tplt[j]);
				cvResetImageROI(gr);
			}

			// draw the (x, y) points and bounding boxes
			val = CV_IMAGE_ELEM(img, unsigned char, y_c, img->nChannels*x_c);
			cvCircle(img, 
				cvPoint(x_c, y_c),
				2,
				val > 150 ? cvScalarAll(BLACK) : cvScalarAll(WHITE),
				CV_FILLED);

			cvRectangle(img, 
				cvPoint(r[j].x, r[j].y), 
				cvPoint(r[j].x + r[j].width, r[j].y + r[j].height),
				CV_RGB(0, 0, 255),
				4);

			memset(text, 0, sizeof(text));
			sprintf(text, "%d", j);
			cvPutText(img, 
				text, 
				cvPoint(x_c, y_c), 
				&font, 
				CV_RGB(0, 255, 0));
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
	cvReleaseImage(&gr);

	return CV_OK;
}

void onclick_center_rect(int e, int x, int y, int flags, void *param)
{
	CvRect *r;

	if(e == CV_EVENT_LBUTTONDOWN) {
		r = (CvRect *) param;
		r[cur_roi].x = x - r[cur_roi].width/2;
		r[cur_roi].y = y - r[cur_roi].height/2;
	}
	else if(e == CV_EVENT_MOUSEMOVE) {
		mouse_loc.x = x;
		mouse_loc.y = y;
	}
}