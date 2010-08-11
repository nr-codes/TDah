#include "t_dah.h"

#define KAL_COLOR CV_RGB(0, 255, 0)
#define CTRD_COLOR CV_RGB(0, 0, 255)
#define TPLT_COLOR CV_RGB(255, 0, 0)

#define POINT_RADIUS 1
#define ROI_THICKNESS 2
#define TXT_SIZE 100
#define X_AXIS 0
#define Y_AXIS 1
#define NUM_AXIS 2
#define AXIS_LENGTH 40

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
	snprintf(text, TXT_SIZE, "%d", i);
	cvPutText(dst, 
		text, 
		cvPoint(ctrd.x - 10, ctrd.y - 6), 
		&font, 
		CTRD_COLOR);

	memset(text, 0, TXT_SIZE);
	snprintf(text, TXT_SIZE, "(%d, %d)", ctrd.x, ctrd.y);
	cvPutText(dst, 
		text, 
		cvPoint(ctrd.x, ctrd.y), 
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
	snprintf(text, TXT_SIZE, "(%d,%d)", pt.x, pt.y);
	cvPutText(dst, text, pt, &font, KAL_COLOR);
}

// TODO: note careful explanation of 3 frames in play here
void draw_axis(IplImage *dst, char *axis_label, CvPoint org, CvMat *R)
{
	int len;
	double angle, dx, dy;
	char text[100];
	CvFont font;
	CvPoint p[NUM_AXIS], q;

	// choose standard length or smaller
	len = std::min((dst->width - org.x), (dst->height - org.y));
	len = std::min(len, AXIS_LENGTH);

	// draw the origins of each coordinate system
	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1);
	for(int i = 0; i < NUM_AXIS; i++) {
		// get axis points
		dx = cvmGet(R, 0, i);
		dy = cvmGet(R, 1, i);
		p[i].x = cvRound(org.x + len*dx);
		p[i].y = cvRound(org.y + len*dy);

		// draw quivers
		dx = p[i].x - org.x;
		dy = p[i].y - org.y;
		angle = atan2(dy, dx);
		q = cvPoint(p[i].x - cvRound(10*cos(angle + CV_PI/4)), 
			p[i].y - cvRound(10*sin(angle + CV_PI/4)));
		cvDrawLine(dst, p[i], q, CV_RGB(0, 0, 255), 2, CV_AA);

		q = cvPoint(p[i].x - cvRound(10*cos(angle - CV_PI/4)), 
			p[i].y - cvRound(10*sin(angle - CV_PI/4)));
		cvDrawLine(dst, p[i], q, CV_RGB(0, 0, 255), 2, CV_AA);

		// draw axis
		cvDrawLine(dst, org, p[i], CV_RGB(0, 0, 255), 2, CV_AA);
		memset(text, 0, sizeof(text));
		snprintf(text, sizeof(text), "%c", axis_label[i]);

		// draw label
		p[i].x = (org.x + p[i].x) / 2;
		p[i].y = (org.y + p[i].y) / 2;
		cvPutText(dst, text, p[i], &font, CV_RGB(0, 255, 0));
	}
}

void draw_wrld2pxl(IplImage *dst, char *axis, CvMat *A, CvMat *k, CvMat *R, CvMat *t)
{
	char text[100];
	CvPoint p;
	CvPoint2D32f w;
	CvFont font;
	int GRID_SPACING = 20;
	p = world2pixel(cvPoint2D32f(0, 0), A, k, R, t);
	draw_axis(dst, axis, p, R);

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.35, 0.35, 0, 1, CV_AA);
	for(int x = GRID_SPACING; x < dst->width - GRID_SPACING; x+=GRID_SPACING) {
		for(int y = GRID_SPACING; y < dst->height - GRID_SPACING; y+=GRID_SPACING) {
			p.x = x;
			p.y = y;
			cvDrawCircle(dst, cvPoint(p.x, p.y), 2, CV_RGB(0,255, 255), CV_FILLED);

			w = pixel2world(p, A, k, R, t);
			p.y += GRID_SPACING/4;
			memset(text, 0, sizeof(text));
			snprintf(text, sizeof(text), "(%0.1g,%0.1g)", w.x, w.y);
			cvPutText(dst, text, p, &font, CV_RGB(0,255,0));
		}
	}
}

void draw_wrld2pxl(IplImage *dst, int rows, int cols,
				   CvMat *wrld_pts, CvMat *prj_wrld_pts, CvMat *img_pts)
{
	char text[100];
	int num_pts;
	CvFont font;
	CvPoint p, w;
	CvPoint2D32f *corners;

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC, 0.5, 0.5, 0, 1);

	// draw the first point found on the checkerboard
	p.x = cvRound(cvmGet(img_pts, 0, 0));
	p.y = cvRound(cvmGet(img_pts, 0, 1));
	cvDrawCircle(dst, p, 6, CV_RGB(255, 0, 0), CV_FILLED);

	// draw the original image points on the checkerboard
	num_pts = rows * cols;
	corners = (CvPoint2D32f *) cvAlloc(num_pts * sizeof(CvPoint2D32f));
	if(!corners) return;

	for(int i = 0; i < num_pts; i++) {
		corners[i].x = (float) cvmGet(img_pts, i, 0);
		corners[i].y = (float) cvmGet(img_pts, i, 1);
	}
	cvDrawChessboardCorners(dst, cvSize(cols, rows), corners, num_pts, true);

	// draw world points to their projected points in the image
	for(int i = 0; i < num_pts; i++) {
		p.x = cvRound(cvmGet(prj_wrld_pts, i, 0));
		p.y = cvRound(cvmGet(prj_wrld_pts, i, 1));
		cvDrawRect(dst, cvPoint(p.x - 10, p.y - 10), 
			cvPoint(p.x + 10, p.y + 10), CV_RGB(128, 128, 128));
		cvDrawCircle(dst, cvPoint(p.x, p.y), 2, CV_RGB(0,255, 255), CV_FILLED);

		w.x = cvRound(cvmGet(wrld_pts, i, 0));
		w.y = cvRound(cvmGet(wrld_pts, i, 1));
		memset(text, 0, sizeof(text));
		snprintf(text, sizeof(text), "(%d,%d)", w.x, w.y);
		cvPutText(dst, text, p, &font, CV_RGB(0,255,0));
	}

	cvFree(&corners);
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

void show_undistorted_image(IplImage *img, CvMat *A, CvMat *k)
{
	// undistortion code
	IplImage *t = cvCloneImage(img);
	IplImage *mapx = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
	IplImage *mapy = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
	if(mapx && mapy && t) {
		cvInitUndistortMap(A, k, mapx, mapy);
		cvShowImage("Distorted Image", img); // Show raw image
		cvRemap(t, img, mapx, mapy); // undistort image
		cvShowImage("Undistorted Image", img); // Show corrected image
	}
	
	cvReleaseImage(&t);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
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
