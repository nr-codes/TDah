//http://www.parashift.com/c++-faq-lite/inline-functions.html

// should warn users that roi 0x0 is bad and pyrdown will divide by roi/2^lvl

#include "t_dah.h"

double track_ctrd(IplImage *gray, int roi_w, int roi_h, 
				  int thresh, CvSeqWriter *wr)
{
	int x, y, w, h;
	int y0;
	char *pxl;
	CvPoint pt;
	float rad;
	CvPoint2D32f cen;
	CvSeq *ptr_seq;

	w = gray->roi->width;
	h = gray->roi->height;
	cvClearSeq(wr->seq);

	// binarize image
	cvThreshold(gray, gray, thresh, WHITE, THRESHOLD_TYPE);

	cvStartAppendToSeq(wr->seq, wr);
	pxl = gray->imageData + gray->roi->yOffset*gray->widthStep + 
		gray->roi->xOffset*gray->nChannels;

	// avoid boundary problem at yOffset = 0
	y0 = gray->roi->yOffset == 0;
	for(x = 0; x < w; x++) {
		for(y = y0; y < h; y++) {
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
		x = cvRound(gray->roi->xOffset + cen.x);
		y = cvRound(gray->roi->yOffset + cen.y);
		ctrd2roi(gray, x, y, roi_w, roi_h);
	}
	else {
		return 0;
	}

	return rad;
}

double track_tmplt_pyr(IplImage *gr, IplImage *tmplt, 
					   IplImage *temp, int lvl, int offset)
{
	double max;
	int x, y, w, h, s;
	IplImage *g_pyr, *t_pyr;

	g_pyr = cvCloneImage(gr);
	t_pyr = cvCloneImage(tmplt);

	s = 1;
	for(int i = 1; i <= lvl; i++) {
		// scale factor
		s = 1 << i;

		// gray image pyramid
		w = gr->width / s;
		h = gr->height / s;
		cvSetImageROI(temp, cvRect(0, 0, w, h));
		cvPyrDown(g_pyr, temp);
		cvSetImageROI(g_pyr, cvRect(0, 0, w, h));
		cvCopyImage(temp, g_pyr);

		// template image pyramid
		x = tmplt->roi->xOffset / s;
		y = tmplt->roi->yOffset / s;
		w = tmplt->roi->width / s;
		h = tmplt->roi->height / s;
		cvSetImageROI(temp, cvRect(temp->width/2, temp->height/2, w, h));
		cvPyrDown(t_pyr, temp); // TODO maybe a bug in fcn when roi = 0 (w or h or both)
		cvSetImageROI(t_pyr, cvRect(x, y, w, h));
		cvCopyImage(temp, t_pyr);
	}

	max = track_tmplt(g_pyr, t_pyr);
	cvSetImageROI(gr, 
			cvRect(g_pyr->roi->xOffset*s, g_pyr->roi->yOffset*s,
					tmplt->roi->width, tmplt->roi->height));

	if(offset) {
		x = gr->roi->xOffset - offset;
		y = gr->roi->yOffset - offset;
		w = tmplt->roi->width + 2*offset;
		h = tmplt->roi->height + 2*offset;

		if(x < 0) {
			// add amount x is -'ve by to x and w
			w = w - x;
			x = 0;
		}

		if(y < 0) {
			// same with y and h
			h = h - y;
			y = 0;
		}

		if(x + w >= gr->width) {
			x = gr->width - w;
		}

		if(y + h >= gr->height) {
			y = gr->height - h;
		}

		cvSetImageROI(gr, cvRect(x, y, w, h));
		max = track_tmplt(gr, tmplt);
	}

	cvReleaseImage(&g_pyr);
	cvReleaseImage(&t_pyr);

	return max;
}

double track_tmplt(IplImage *gray, IplImage *templ, IplImage *temp)
{
	IplImage *res;
	CvPoint min_pt, max_pt;
	CvRect gr_roi, t_roi;
	double min, max;

	gr_roi = cvGetImageROI(gray);
	t_roi = cvGetImageROI(templ);

	if(temp) {
		res = temp;
		cvSetImageROI(res, cvRect(0, 0, gr_roi.width - t_roi.width + 1, 
			gr_roi.height - t_roi.height + 1));
	}
	else {
		res = cvCreateImage(cvSize(gr_roi.width - t_roi.width + 1, 
					gr_roi.height - t_roi.height + 1), IPL_DEPTH_32F, 1);
	}

	cvMatchTemplate(gray, templ, res, CV_TM_CCOEFF_NORMED);
	cvMinMaxLoc(res, &min, &max, &min_pt, &max_pt);

	gr_roi.x += max_pt.x;
	gr_roi.y += max_pt.y;
	gr_roi.width = t_roi.width;
	gr_roi.height = t_roi.height;

	cvSetImageROI(gray, gr_roi);
	if(temp == NULL) {
		cvReleaseImage(&res);
	}
	
	return max;
}