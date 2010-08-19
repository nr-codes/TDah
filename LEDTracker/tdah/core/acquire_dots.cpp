#include "t_dah.h"

// TODO note that if rect is out of image bounds OpenCV resizes ImageROI
// to largest valid bounding box

#define AUTOACQ_NUM_ATTEMPTS 20

//***************************** HELPER FUNCTIONS *********************//

static IplImage *cvt_bgr2gr(IplImage *img, IplImage *gr)
{
	CvRect roi;
	int is_roi_set;

	is_roi_set = !!gr->roi;
	
	roi = cvGetImageROI(gr);
	cvResetImageROI(gr);
	cvCvtColor(img, gr, CV_BGR2GRAY);
	cvCvtColor(gr, img, CV_GRAY2BGR);

	if(is_roi_set) cvSetImageROI(gr, roi);

	return img;
}

static void update_kal_tplt(IplImage *gr, int roi_w, int roi_h, 
					 CvKalman *kal, IplImage *tplt)
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

		estimate_and_predict(kal, (float) dt_k, z_k);
		kal_assert(gr, kal->state_post, roi_w, roi_h);
		ctrd2roi(gr, 
			cvRound(kal->state_post->data.fl[0]), 
			cvRound(kal->state_post->data.fl[1]),
			roi_w, 
			roi_h);
		dt_k = cvGetTickCount()/cvGetTickFrequency();
	}

	// update template
	if(tplt != NULL) {
		cvSetImageROI(tplt, cvGetImageROI(gr));
		cvCopyImage(gr, tplt);
	}
}

//***************************** AUTO CALIBRATION *********************//

int auto_acquire(CvCapture *capture, IplImage **gr, 
				 int roi_w, int roi_h, int t, int thresh_type, CvSeqWriter *wr, 
				 double r, int n, IplImage **tplt, double m, 
				 CvKalman **kal, int show_results)
{
	double score;
	int found;
	int last_tplt;
	IplImage *img;
	int tries = AUTOACQ_NUM_ATTEMPTS;
	CvPoint2D32f c;

	while(tries--) {
		// reset state
		found = 0;
		score = 0;
		last_tplt = -1;

		// take a new image
		img = cvt_bgr2gr(cvQueryFrame(capture), gr[0]);

		// find dots
		for(int i = 0; i < n; i++) {
			img = cvt_bgr2gr(img, gr[i]);
			if(gr[i]->roi) {
				score = track_ctrd(gr[i], roi_w, roi_h, t, thresh_type, &wr[i], &c);
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

					// copy orig img to gr[i] for tmplt matching
					cvt_bgr2gr(img, gr[i]);
					update_kal_tplt(gr[i], roi_w, roi_h,
						kal ? kal[i] : NULL, tplt ? tplt[i] : NULL);
				}
			}
			else if(tplt && tplt[i]) {
				cvt_bgr2gr(img, gr[i]);
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

		// have all dots been found?
		if(found == n) {
			break;
		}
	}

	if(show_results) {
		for(int i = 0; i < n; i++) {
			if(gr[i]->roi) {
				if(kal && kal[i]) {
					draw_kal(img, kal[i]);
				}

				draw_ctrd(img, gr[i], wr[i].seq, i);
			}
		}
		cvShowImage("auto_acquire", img);
		cvWaitKey(0);
		cvDestroyWindow("auto_acquire");
	}

	return (found == n) ? CV_OK : !CV_OK;
}

//***************************** MANUAL CALIBRATION *********************//

// shared variables between manual_acquire & mouse callback function
static int cur_roi;
static CvPoint mouse_loc;
static int roi_width, roi_height;

void onclick_center_rect(int e, int x, int y, int flags, void *param)
{
	IplImage **gr;

	if(e == CV_EVENT_LBUTTONDOWN) {
		gr = (IplImage **) param;
		ctrd2roi(gr[cur_roi], x, y, roi_width, roi_height);
	}
	else if(e == CV_EVENT_MOUSEMOVE) {
		mouse_loc.x = x;
		mouse_loc.y = y;
	}
}

int manual_acquire(CvCapture *capture, IplImage **gr, int roi_w, int roi_h,
				   int *t, int t_type, CvSeqWriter *wr, int n, 
				   IplImage **tplt, CvKalman **kal)
{
	int rc;
	CvFont font;
	IplImage *img, *gr_temp;
	char text[100];
	CvPoint2D32f c;

	// TODO: write asserts
	
	roi_width = roi_w;
	roi_height = roi_h;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	cur_roi = 0;
	img = cvQueryFrame(capture);
	gr_temp = cvCloneImage(gr[0]);

	if(*t < BLACK && *t > WHITE) {
		*t = THRESHOLD;
	}

	cvNamedWindow("manual_acquire");
	cvCreateTrackbar("manual_acquire_track", "manual_acquire", t, WHITE, NULL);
	cvSetMouseCallback("manual_acquire", onclick_center_rect, gr);

	for(int j = 0; j < n; j++) {
		cvResetImageROI(gr[j]);
	}

	while(1) {
		img = cvt_bgr2gr(cvQueryFrame(capture), gr[cur_roi]);

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
				THRESHOLD_TYPE);
		}

		for(int j = 0; j < n; j++) {
			if(gr[j]->roi == NULL) {
				continue;
			}

			// preserve gr[j] for tmplt copying
			cvSetImageROI(gr_temp, cvGetImageROI(gr[j]));
			cvCopyImage(gr[j], gr_temp);

			track_ctrd(gr_temp, roi_width, roi_height, *t, t_type, &wr[j], &c);
			cvSetImageROI(gr[j], cvGetImageROI(gr_temp));

			update_kal_tplt(gr[j], roi_width, roi_height,
					kal ? kal[j] : NULL, 
					tplt ? tplt[j] : NULL);

			if(kal && kal[j]) {
				draw_kal(img, kal[j]);
			}

			draw_ctrd(img, gr[j], wr[j].seq, j);
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

	for(int j = 0; j < n; j++) {
		if(gr[j]->roi == NULL) return !CV_OK;
	}

	cvDestroyWindow("manual_acquire");
	cvReleaseImage(&gr_temp);
	return CV_OK;
}