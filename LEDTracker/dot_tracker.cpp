//http://www.parashift.com/c++-faq-lite/inline-functions.html

#include "t_dah.h"

int position(IplImage *gray, CvRect *rect, int thresh, CvSeqWriter *wr)
{
	double time_us;
	int x, y, w, h;
	char *pxl;
	LARGE_INTEGER start, stop, freq;
	CvPoint pt;
	float rad;
	CvPoint2D32f cen;
	CvSeq *ptr_seq;

	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);

	w = rect->width;
	h = rect->height;
	cvClearSeq(wr->seq);

	// binarize image
	cvThreshold(gray, gray, thresh, WHITE, CV_THRESH_BINARY_INV);

	cvStartAppendToSeq(wr->seq, wr);
	pxl = gray->imageData + gray->roi->yOffset*gray->widthStep + 
		gray->roi->xOffset*gray->nChannels;
	for(x = 0; x < w; x++) {
		for(y = 0; y < h; y++) {
			// does I(x, y) = WHITE && dI(x, y)/dy != 0
			if(pxl[x*gray->nChannels + (y * gray->widthStep)] &&
				pxl[x*gray->nChannels + ((y-1) * gray->widthStep)] -
				pxl[x*gray->nChannels + ((y+1) * gray->widthStep)]) {
				// we've found a pixel on the obj boundary
				pt.x = x;
				pt.y = y;
				CV_WRITE_SEQ_ELEM(pt, *wr);
			}
		}
	}

	ptr_seq = cvEndWriteSeq(wr);
	if(ptr_seq->total && cvMinEnclosingCircle(ptr_seq, &cen, &rad)) {
		x = gray->roi->xOffset + cvRound(cen.x);
		y = gray->roi->yOffset + cvRound(cen.y);

		rect->x = x - w/2;
		rect->y = y - h/2;

		if(rad > 20) {
			return !CV_OK;
		}
	}
	else {
		return !CV_OK;
	}
	QueryPerformanceCounter(&stop);

	time_us = (stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
	printf("pos: %g --- ", time_us);
	printf("%g (%d, %d)\n", rad, x, y);

	return CV_OK;
}

int emergency(CvRect *rect, IplImage *gray, IplImage *templ)
{
	IplImage *res;
	CvPoint min_pt, max_pt;
	int w, h;
	double min, max;
	double time_us, match;
	LARGE_INTEGER start, stop, freq;

	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	w = rect->width;
	h = rect->height;

	res = cvCreateImage(
			cvSize(gray->width - w + 1, gray->height - h + 1), 
			IPL_DEPTH_32F, 1);

	cvResetImageROI(gray);
	cvMatchTemplate(gray, templ, res, CV_TM_CCOEFF_NORMED);
	cvMinMaxLoc(res, &min, &max, &min_pt, &max_pt);

	rect->x = max_pt.x;
	rect->y = max_pt.y;

	// NOTE: how fast?
	cvSetImageROI(gray, *rect);	
	match = cvMatchShapes(templ, gray, CV_CONTOURS_MATCH_I1);
	cvReleaseImage(&res);
	QueryPerformanceCounter(&stop);
	
	time_us = 
	(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
	printf("emerg: %g us min: %g max: %g match: %g\n", time_us, min, max, match);

	return CV_OK;
}


// modify so window is specified or temp is returned
void draw_position(IplImage *gray, IplImage *rgb, CvSeq *pts, CvKalman *kal)
{
	IplImage *temp;
	char text[100];
	CvFont font;
	CvPoint2D32f cen;
	float rad;

	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1,1,0,1);

	temp = cvCloneImage(gray);
	cvZero(temp);

	for(int i = 0; i < pts->total; i++) {
		CvPoint *p = (CvPoint *) cvGetSeqElem(pts, i);
		(temp->imageData + 
					gray->roi->yOffset*gray->widthStep + 
					gray->roi->xOffset)
						[p->x + (p->y * gray->widthStep)] = (char) 255;

	}

	// draw min enclosing
	if(pts->total && cvMinEnclosingCircle(pts, &cen, &rad)) {
		cvCircle(rgb, cvPoint(cvRound(cen.x), cvRound(cen.y)), cvRound(rad), 
			CV_RGB(0,0,255), 1);
	}

	cvResetImageROI(rgb);

	// draw kalman
	cvCircle(rgb, 
		cvPoint(cvRound(kal->state_post->data.fl[0]), 
				cvRound(kal->state_post->data.fl[1])),
		2, CV_RGB(255,0,0), -1);

	memset(text, 0, 100);
	sprintf_s(text, 100, "(%d,%d)", gray->roi->xOffset, gray->roi->yOffset);
	cvPutText(rgb, text, cvPoint(gray->roi->xOffset, gray->roi->yOffset), 
		&font, CV_RGB(0,255,0));

	cvShowImage("rgb", rgb);
	cvShowImage("gray", gray);
	cvShowImage("temp", temp);

	cvReleaseImage(&temp);
}
