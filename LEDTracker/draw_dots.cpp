#include "t_dah.h"

#define KAL_COLOR CV_RGB(0, 255, 0)
#define CTRD_COLOR CV_RGB(0, 0, 255)
#define TPLT_COLOR CV_RGB(255, 0, 0)

#define POINT_RADIUS 1
#define ROI_THICKNESS 2
#define TXT_SIZE 100

void draw_ctrd(IplImage *dst, IplImage *src, CvSeq *bndry, int i)
{
	CvPoint ctrd;
	CvPoint2D32f ctr;
	float radius;
	int w, h;
	char text[TXT_SIZE];
	CvFont font;

	if(src->roi == NULL) {
		// roi isn't being used so ignore img
		return;
	}

	w = src->roi->width;
	h = src->roi->height;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	if(bndry == NULL || bndry->total <= 0 ||
		!cvMinEnclosingCircle(bndry, &ctr, &radius)) {
		radius = POINT_RADIUS;
	}
	
	// use the src image xoffset and yoffset
	ctrd = roi2ctrd(src);
	cvCircle(dst, ctrd,	cvRound(radius), CTRD_COLOR);

	cvRectangle(dst, 
		cvPoint(ctrd.x - w/2, ctrd.y - h/2), 
		cvPoint(ctrd.x + w/2, ctrd.y + h/2),
		CTRD_COLOR,
		ROI_THICKNESS);

	memset(text, 0, TXT_SIZE);
	sprintf_s(text, TXT_SIZE, "%d", i);
	cvPutText(dst, 
		text, 
		cvPoint(ctrd.x - 10, ctrd.y - 6), 
		&font, 
		CTRD_COLOR);
}

void draw_kal(IplImage *dst, CvKalman *kal)
{
	char text[TXT_SIZE];
	CvFont font;
	CvPoint pt;

	if(kal == NULL) {
		return;
	}

	cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.6, 0.6);

	// get current state
	pt = cvPoint(cvRound(kal->state_post->data.fl[0]), 
				cvRound(kal->state_post->data.fl[1]));

	// draw kalman
	cvCircle(dst, pt, POINT_RADIUS, KAL_COLOR, CV_FILLED);

	memset(text, 0, TXT_SIZE);
	sprintf_s(text, TXT_SIZE, "(%d,%d)", pt.x, pt.y);
	cvPutText(dst, text, pt, &font, KAL_COLOR);
}

void show_tplts(IplImage **tplt, int roi_w, int roi_h, int rows, int cols, int n)
{
	char text[TXT_SIZE];
	CvFont font;
	int w, h, i;
	IplImage *dst;

	if(tplt == NULL || rows*cols != n) {
		return;
	}

	dst = cvCreateImage(cvSize(roi_w*cols, roi_h*rows), 
		IPL_DEPTH_8U, 3);

	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	i = 0;
	for(int r = 0; r < rows; r++) {
		for(int c = 0; c < cols; c++) {
			if(tplt[i]) {
				w = tplt[i]->roi->width;
				h = tplt[i]->roi->height;

				cvSetImageROI(dst, cvRect(c*w, r*h, w, h));

				if(tplt[i]->nChannels != 3) {
					cvCvtColor(tplt[i], dst, CV_GRAY2BGR);
				}
				else {
					cvCopyImage(tplt[i], dst);
				}

				memset(text, 0, TXT_SIZE);
				sprintf_s(text, TXT_SIZE, "%d", i);
				cvPutText(dst, text, cvPoint(w/2, h/2), 
					&font, TPLT_COLOR);
			}

			i++;
		}
	}

	cvResetImageROI(dst);
	cvShowImage("templates", dst);
	cvReleaseImage(&dst);
}

void show_seqs(CvSeqWriter *bndry, int roi_w, int roi_h, int rows, int cols, int n)
{
	char text[TXT_SIZE];
	CvFont font;
	int w, h, i;
	int x, y;
	IplImage *dst;
	CvPoint *p;

	if(bndry == NULL || rows*cols != n) {
		return;
	}

	w = roi_w;
	h = roi_h;
	dst = cvCreateImage(cvSize(w*cols, h*rows), 
		IPL_DEPTH_8U, 3);
	cvZero(dst);

	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	i = 0;
	for(int r = 0; r < rows; r++) {
		for(int c = 0; c < cols; c++) {
			if(bndry[i].seq) {
				for(int j = 0; j < bndry[i].seq->total; j++) {
					p = (CvPoint *) cvGetSeqElem(bndry[i].seq, j);
					x = 3*(p->x + c*w);
					y = p->y + r*h;

					CV_IMAGE_ELEM(dst, unsigned char, y, x) = WHITE;
					CV_IMAGE_ELEM(dst, unsigned char, y, x + 1) = WHITE;
					CV_IMAGE_ELEM(dst, unsigned char, y, x + 2) = WHITE;
				}

				memset(text, 0, TXT_SIZE);
				sprintf_s(text, TXT_SIZE, "%d", i);
				cvPutText(dst, text, cvPoint(c*w + w/2, r*h + h/2), 
					&font, CTRD_COLOR);
			}

			i++;
		}
	}

	cvShowImage("sequences", dst);
	cvReleaseImage(&dst);
}

void show_position(IplImage **roiImg, int n, 
				   CvKalman **kal, CvSeqWriter *bndry, char *draw_what,
				   IplImage *fullImg)
{
	IplImage *dst;

	if(fullImg == NULL) {
		dst = cvCreateImage(
			cvSize(roiImg[0]->width, roiImg[0]->height), IPL_DEPTH_8U, 3);
		cvZero(dst);
	}
	else {
		dst = fullImg;
	}

	for(int i = 0; i < n; i++) {
		if(draw_what && draw_what[i]) {

		}
		else {
			draw_ctrd(dst, roiImg[i], bndry ? bndry[i].seq : NULL, i);
		}

		if(kal) {
			draw_kal(dst, kal[i]);
		}
	}

	cvShowImage("position", dst);
	if(fullImg == NULL) {
		cvReleaseImage(&dst);
	}
}
