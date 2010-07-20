//http://www.parashift.com/c++-faq-lite/inline-functions.html

#include "t_dah.h"

double track_ctrd(IplImage *gray, int thresh, CvSeqWriter *wr)
{
	int x, y, w, h;
	char *pxl;
	CvPoint pt;
	float rad;
	CvPoint2D32f cen;
	CvSeq *ptr_seq;

	w = gray->roi->width;
	h = gray->roi->height;
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
		ctrd2roi(gray, x, y, ROI_WIDTH, ROI_HEIGHT);
	}
	else {
		return 0;
	}

	return rad;
}

double track_tmplt_pyr(IplImage *gr, IplImage *tmplt, int lvl)
{
	double max;
	int x, y, w, h, s;
	IplImage *temp, *g_pyr, *t_pyr;

	g_pyr = cvCloneImage(gr);
	t_pyr = cvCloneImage(tmplt);
	temp = cvCreateImage(
			cvSize(gr->width, gr->height), 
			gr->depth, gr->nChannels);

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
		//cvSetImageROI(temp, cvRect(gr->width/2, gr->height/2, w, h));
		cvSetImageROI(temp, cvRect(0, 0, w, h));
		cvPyrDown(t_pyr, temp);
		cvSetImageROI(t_pyr, cvRect(x, y, w, h));
		cvCopyImage(temp, t_pyr);
	}

	max = track_tmplt(g_pyr, t_pyr);
	cvSetImageROI(gr, 
			cvRect(g_pyr->roi->xOffset*s,
					g_pyr->roi->yOffset*s,
					g_pyr->roi->width*s,
					g_pyr->roi->height*s));

	cvReleaseImage(&g_pyr);
	cvReleaseImage(&t_pyr);
	cvReleaseImage(&temp);

	return max;
}

double track_tmplt_pyr2(IplImage *gr, IplImage *tmplt, int lvl)
{
	double max;
	IplImage *gr_pyr, *tmplt_pyr;

	if(lvl) {
		gr_pyr = cvCreateImage(
			cvSize(gr->width/2, gr->height/2), 
			gr->depth, gr->nChannels);

		tmplt_pyr = cvCreateImage(
			cvSize(tmplt->width/2, tmplt->height/2), 
			tmplt->depth, tmplt->nChannels);

		cvSetImageROI(tmplt_pyr, 
			cvRect(tmplt->roi->xOffset/2, tmplt->roi->yOffset/2,
			tmplt->roi->width/2, tmplt->roi->height/2));

		cvPyrDown(gr, gr_pyr);
		cvPyrDown(tmplt, tmplt_pyr);

		max = track_tmplt_pyr(gr_pyr, tmplt_pyr, lvl - 1);
		cvSetImageROI(gr, 
			cvRect(gr_pyr->roi->xOffset*2,
					gr_pyr->roi->yOffset*2,
					gr_pyr->roi->width*2,
					gr_pyr->roi->height*2));

		cvReleaseImage(&gr_pyr);
		cvReleaseImage(&tmplt_pyr);
	}
	else {
		max = track_tmplt(gr, tmplt);
	}
	
	return max;
}

double track_tmplt(IplImage *gray, IplImage *templ)
{
	IplImage *res;
	CvPoint min_pt, max_pt;
	CvRect gr_roi, t_roi;
	double min, max;

	gr_roi = cvGetImageROI(gray);
	t_roi = cvGetImageROI(templ);

	res = cvCreateImage(cvSize(gr_roi.width - t_roi.width + 1, 
				gr_roi.height - t_roi.height + 1), IPL_DEPTH_32F, 1);

	cvMatchTemplate(gray, templ, res, CV_TM_CCOEFF_NORMED);
	cvMinMaxLoc(res, &min, &max, &min_pt, &max_pt);
	cvSetImageROI(gray, cvRect(max_pt.x, max_pt.y, t_roi.width, t_roi.height));
	cvReleaseImage(&res);
	
	return max;
}