#include "cv.h"
#include "cxcore.h"
#include "highgui.h"


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


int position(IplImage *gray, CvSeqWriter *wr, int w, int h, 
			 CvKalman *kal, CvMat *z_k, CvMat *u_k)
{
	double time_us;
	int x, y;
	char *pxl;
	LARGE_INTEGER start, stop, freq;
	CvPoint pt;
	float rad;
	CvPoint2D32f cen;
	CvSeq *ptr_seq;

	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	cvClearSeq(wr->seq);

	// binarize image
	cvThreshold(gray, gray, 85, 255, CV_THRESH_BINARY_INV);

	cvStartAppendToSeq(wr->seq, wr);
	pxl = gray->imageData + gray->roi->yOffset*gray->widthStep + gray->roi->xOffset;
	for(x = 0; x < w; x++) {
		for(y = 0; y < h; y++) {
			// does I(x, y) = WHITE && dI(x, y)/dy != 0
			if(pxl[x + (y * gray->widthStep)] &&
				pxl[x + ((y-1) * gray->widthStep)] -
				pxl[x + ((y+1) * gray->widthStep)]) {
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

		cvKalmanPredict(kal, u_k);
		// need to add v_k?
		z_k->data.fl[0] = (float) x;
		z_k->data.fl[1] = (float) y;
		cvKalmanCorrect(kal, z_k);

		cvSetImageROI(gray, cvRect(x - w/2, y - h/2, w, h));
	}
	else {
		// do different tracking alg.
		rad = 0;
		cen.x = 0;
		cen.y = 0;
	}
	QueryPerformanceCounter(&stop);

	time_us = (stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
	printf("%g (%g) --- ", time_us, time_us / (gray->width * gray->height));
	printf("%g (%d, %d) <==> (%g, %g)\n", rad, x, y, 
		kal->state_post->data.fl[0], kal->state_post->data.fl[1]);

	return 0;
}

int emerg1(CvCapture *capture)
{
	IplImage *rgb = cvQueryFrame(capture);
	IplImage *frame1 = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	IplImage *frame2 = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	IplImage *diff = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);

	cvCvtColor(rgb, frame1, CV_BGR2GRAY);

	rgb = cvQueryFrame(capture);
	cvCvtColor(rgb, frame2, CV_BGR2GRAY);
	
	cvAbsDiff(frame1, frame2, diff);
	cvThreshold(diff, diff, 30, 255, CV_THRESH_BINARY);

	cvShowImage("diff", diff);

	return 0;
}

void CvMouseCall(int event0, int x, int y, int flags, void *params)
{
	if(event0 == CV_EVENT_LBUTTONDOWN) {
		IplImage **imgs = (IplImage **) params;
		IplImage *f1 = imgs[0];
		IplImage *templ = imgs[1];

		cvSetImageROI(f1, cvRect(x-templ->width/2, 
			y-templ->height/2, templ->width, templ->height));
		cvCopyImage(f1, templ);
		cvResetImageROI(f1);
	}
}

int main(int argc, char *argv[])
{
	CvCapture *capture = NULL;
	IplImage *rgb, *gray;

	capture = cvCaptureFromCAM(0);
	if(!capture) {
		printf("init_cam_src: can't open camera\n");
		return -1;
	}

	rgb = cvQueryFrame(capture);
	gray = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);

	int w = 40;
	int h = 40;
	int x = 301;
	int y = 172;
	cvSetImageROI(gray, cvRect(x-w/2, y-h/2, w, h));

	float dt = 0.1f;
	float g = 0.0f;
	const float A[] = {1, 0, dt, 0, 
						0, 1, 0, dt, 
						0, 0, 1, 0,
						0, 0, 0, 1};
	const float B[] = {0, 0.5f*g*dt, 0, g*dt};
	const float H[] = {1, 0, 0, 0,
						0, 1, 0, 0};
	CvKalman *kal = cvCreateKalman(4, 2, 1);
	CvMat* x_k = cvCreateMat(4, 1, CV_32FC1);
    CvMat* w_k = cvCreateMat(4, 1, CV_32FC1);
	CvMat* z_k = cvCreateMat(2, 1, CV_32FC1);
	cvZero(z_k);
	CvMat* u_k = cvCreateMat(1, 1, CV_32FC1);
	cvSetIdentity(u_k, cvRealScalar(1));

	memcpy( kal->transition_matrix->data.fl, A, sizeof(A));
	memcpy( kal->control_matrix->data.fl, B, sizeof(B));
	memcpy( kal->measurement_matrix->data.fl, H, sizeof(H));
	cvSetIdentity( kal->process_noise_cov, cvRealScalar(1e-1));
	cvSetIdentity( kal->measurement_noise_cov, cvRealScalar(1e-5));
	cvSetIdentity( kal->error_cov_post, cvRealScalar(1));
	kal->state_post->data.fl[0] = (float) x;
	kal->state_post->data.fl[1] = (float) y;

	CvSeqWriter wr;
	CvMemStorage *mem = cvCreateMemStorage(0);
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);

	#define ftr_sz 500
	IplImage *eig = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_32F, 1);
	IplImage *temp = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_32F, 1);
	int cnt = ftr_sz;
	int win_size = 10;
	CvSize pyr_size = cvSize(rgb->width + 8, rgb->height/3);
	CvPoint2D32f corns1[ftr_sz];
	CvPoint2D32f corns2[ftr_sz];
	char features_fnd[ftr_sz];
	float feature_errs[ftr_sz];

	IplImage *f1 = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	IplImage *f1_pyr = cvCreateImage(cvSize(rgb->width/2, rgb->height/2), IPL_DEPTH_8U, 1);
	IplImage *f2 = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	IplImage *pyr1 = cvCreateImage(cvSize(rgb->width/2, rgb->height/2), IPL_DEPTH_32F, 1);
	IplImage *pyr2 = cvCreateImage(cvSize(rgb->width/2, rgb->height/2), IPL_DEPTH_32F, 1);

	IplImage *rgb2 = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 3);
	int strt = 0;

	char do_feats = 1;

	CvSize b_sz = cvSize(rgb->width/2, rgb->height/2);
	CvSize sh_sz = cvSize(rgb->width/4, rgb->height/4);
	CvSize max_sz = cvSize(10, 10);
	CvSize w_sz = cvSize(15, 15);

	IplImage *vx = cvCreateImage(
		cvSize((f1->width - b_sz.width)/sh_sz.width, 
		(f1->height - b_sz.height)/sh_sz.height), 
		IPL_DEPTH_32F, 1);
	IplImage *vy = cvCreateImage(
		cvSize((f1->width - b_sz.width)/sh_sz.width, 
		(f1->height - b_sz.height)/sh_sz.height), 
		IPL_DEPTH_32F, 1);

	LARGE_INTEGER start, stop, freq;
	QueryPerformanceFrequency(&freq);

	IplImage *imgs[2];
	IplImage *templ = cvCreateImage(cvSize(w, h), 8, 1);
	IplImage *res = cvCreateImage(
		cvSize(f1->width - w + 1, f1->height - h + 1), 32, 1);
	cvNamedWindow("rgb");
	imgs[0] = f1;
	imgs[1] = templ;
	cvSetMouseCallback("rgb", CvMouseCall, imgs);

	//cvResetImageROI(gray);
	cvZero(templ);
	while(cvWaitKey(100) != 'q') {
		break;
		// grab an image
		rgb = cvQueryFrame(capture);	
		cvCvtColor(rgb, f1, CV_BGR2GRAY);
		CvPoint min_pt, max_pt;
		double min, max;

		int type = CV_TM_CCOEFF_NORMED;

		QueryPerformanceCounter(&start);
		cvMatchTemplate(f1, templ, res, type);
		cvMinMaxLoc(res, &min, &max, &min_pt, &max_pt);
		
		if(type > 1) {
			min_pt = max_pt;
		}

		cvSetImageROI(f1, cvRect(min_pt.x, min_pt.y, w, h));
		double match = cvMatchShapes(templ, f1, CV_CONTOURS_MATCH_I1);
		cvShowImage("f1", f1);
		cvResetImageROI(f1);
		QueryPerformanceCounter(&stop);
		
		double time_us = 
		(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
		printf("%g us min: %g max: %g match: %g\n", time_us, min, max, match);
	

		cvDrawRect(rgb, min_pt, 
			cvPoint(min_pt.x + w, min_pt.y + h), CV_RGB(0,0,255), 3);
		cvCircle(rgb, cvPoint(min_pt.x + w/2, min_pt.y + h/2), 
			2, CV_RGB(255,0,0), -2);

		cvShowImage("rgb", rgb);
		cvShowImage("templ", templ);
		cvShowImage("res", res);
	}
	cvDestroyWindow("rgb");
	cvDestroyWindow("templ");
	cvDestroyWindow("res");

	cvResetImageROI(gray);
	while(cvWaitKey(100) != 'q') {
		break;
		// grab an image
		rgb = cvQueryFrame(capture);	
		cvCvtColor(rgb, f1, CV_BGR2GRAY);

		QueryPerformanceCounter(&start);
		cvPyrDown(f1, f1_pyr);
		cvPyrUp(f1_pyr, f1);
		cvCanny(f1, f1, 15, 100);

		CvSeq *seq;
		cvFindContours(f1, mem, &seq);
		QueryPerformanceCounter(&stop);
		
		double time_us = 
		(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
		printf("%g\n", time_us);

		cvDrawContours(rgb, seq, CV_RGB(0, 255,0), CV_RGB(255,0,0), 3);

		cvShowImage("rgb", rgb);
		cvShowImage("f1", f1);
	}


	cvResetImageROI(gray);
	CvMat *flow = cvCreateMat(rgb->height, rgb->width, CV_32FC2);
	while(cvWaitKey(100) != 'q') {
		break;
		// grab an image
		rgb = cvQueryFrame(capture);	
		cvCvtColor(rgb, f1, CV_BGR2GRAY);

		rgb = cvQueryFrame(capture);
		cvCvtColor(rgb, f2, CV_BGR2GRAY);

		QueryPerformanceCounter(&start);
		cnt = ftr_sz;
		cvGoodFeaturesToTrack(f1, eig, temp, corns1, &cnt, .01, 10);

		cvFindCornerSubPix(f1, corns1, cnt, 
			cvSize(5, 5), cvSize(-1, -1), 
			cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

		cvCalcOpticalFlowPyrLK(f1, f2, pyr1, pyr2, corns1, corns2, cnt, 
				cvSize(win_size,win_size), 7, features_fnd, feature_errs, 
				cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3), 0);
		QueryPerformanceCounter(&stop);

		/*
		QueryPerformanceCounter(&start);
		cvCalcOpticalFlowFarneback(f1, f2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
		QueryPerformanceCounter(&stop);
		*/

		double time_us = 
		(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
		printf("%g\n", time_us);

		cvZero(gray);
		for(int i = 0; i < cnt; i++) {
			CvPoint pt1 = cvPoint(cvRound(corns1[i].x), cvRound(corns1[i].y));
			CvPoint pt2 = cvPoint(cvRound(corns2[i].x), cvRound(corns2[i].y));


			if(pt2.x < 0 || pt2.x >= rgb->width || pt2.y < 0 || 
				pt2.y >= rgb->height) {
				//printf("out of bounds\n");
				continue;
			}



			double dist = sqrt(pow(corns2[i].x-corns1[i].x, 2) + 
				pow(corns2[i].y-corns1[i].y, 2));
			if(features_fnd[i] && dist > 1) {
				cvCircle(rgb, pt1, 2, 
					CV_RGB(255,0,0), -1);

				cvCircle(rgb, pt2, 2, 
					CV_RGB(0,255,0), 1);


				(gray->imageData)
						[pt2.x + (pt2.y * gray->widthStep)] = (char) 255;
			}
		}

		cvShowImage("rgb", rgb);
		cvShowImage("gray", gray);
	}


	
	while(cvWaitKey(100) != 'q') {
		break;
		rgb = cvQueryFrame(capture);
		cvCvtColor(rgb, f2, CV_BGR2GRAY);

		if(strt) {
			QueryPerformanceCounter(&start);
			cvCalcOpticalFlowLK(f1, f2, w_sz, pyr1, pyr2);
			strt = 2;
			QueryPerformanceCounter(&stop);

			double time_us = 
			(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
			for(int j = 0; j < pyr2->height; j++) {
				for(int i = 0; i < pyr2->width; i++) {
					double speed = sqrt(pow(cvGetReal2D(pyr1, j, i), 2) +
						pow(cvGetReal2D(pyr2, j, i), 2));

					if(speed > 20 && speed < 35) {
						/*printf("%g -- (%d, %d): vx: %g vy: %g spd: %g\n", time_us, j, i,
							cvGetReal2D(pyr1, j, i),
							cvGetReal2D(pyr2, j, i), speed);
						*/

						CvPoint pt1 = cvPoint(j, i);
						CvPoint pt2 = cvPoint(i + cvRound(cvGetReal2D(pyr1, j, i)), 
							j + cvRound(cvGetReal2D(pyr2, j, i)));
						

						cvCircle(rgb, pt1, 2, CV_RGB(255,0,0), -1);
						cvCircle(rgb, pt2, 2, CV_RGB(0,255,0), -1);
						cvLine(rgb, pt1, pt2, CV_RGB(0,0,255));

					}
				}
			}
		}

		cvShowImage("rgb", rgb);
		cvShowImage("f1", f1);
		cvShowImage("f2", f2);
		cvShowImage("vx", pyr1);
		cvShowImage("vy", pyr2);

		cvCopyImage(f2, f1);

		if(!strt) {
			strt = 1;
		}
	}


	while(cvWaitKey(100) != 'q') {
		break;
		rgb = cvQueryFrame(capture);
		cvCvtColor(rgb, f2, CV_BGR2GRAY);

		if(strt) {
			QueryPerformanceCounter(&start);

			cvCalcOpticalFlowBM(f1, f2, b_sz, 
				sh_sz, max_sz,
				0/*strt - 1*/, vx, vy);
			strt = 2;

			QueryPerformanceCounter(&stop);
			double time_us = 
			(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
			for(int j = 0; j < vx->height; j++) {
				for(int i = 0; i < vx->width; i++) {
					printf("%g -- (%d, %d): vx: %g vy: %g\n", time_us, j, i,
						cvGetReal2D(vx, j, i),
						cvGetReal2D(vy, j, i));
				}
			}
		}

		cvCopyImage(f2, f1);

		if(!strt) {
			strt = 1;
		}

		cvShowImage("f1", f1);
		cvShowImage("f2", f2);
		cvShowImage("vx", vx);
		cvShowImage("vy", vy);
	}

	while(cvWaitKey(100) != 'q') {
		break;
		
		//emerg1(capture);

		// grab an image
		rgb = cvQueryFrame(capture);
		cvCopyImage(rgb, rgb2);
		
		cvShowImage("gray", gray);
		cvShowImage("rgb", rgb);
		

		if(cvWaitKey(10)) {
			cvSetImageROI(rgb, cvRect(gray->roi->xOffset, gray->roi->yOffset,
				gray->roi->width, gray->roi->height));

			cvSetImageROI(eig, cvRect(gray->roi->xOffset, gray->roi->yOffset,
				gray->roi->width, gray->roi->height));
			cvSetImageROI(temp, cvRect(gray->roi->xOffset, gray->roi->yOffset,
				gray->roi->width, gray->roi->height));

			cvCvtColor(rgb, gray, CV_BGR2GRAY);

			cnt = ftr_sz;
			cvGoodFeaturesToTrack(gray, eig, temp, corns1, &cnt, 0.1, .1);

			cvFindCornerSubPix(gray, corns1, cnt, 
				cvSize(5, 5), cvSize(-1, -1), 
				cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

			for(int i = 0; i<cnt;i++) {
				corns1[i].x += gray->roi->xOffset;
				corns1[i].y += gray->roi->yOffset;
				cvCircle(rgb2, cvPoint((int)corns1[i].x, (int)corns1[i].y), 2, 
					CV_RGB(255,0,0), -1);
				//printf("%d: x: %g y: %g e: %g\n", i,
				//	corns1[i].x, corns1[i].y, cvGetReal1D(eig, i));
			}
			cvShowImage("rgb2", rgb2);

			cvResetImageROI(rgb);

			cvCvtColor(rgb, f1, CV_BGR2GRAY);

			LARGE_INTEGER start, stop, freq;
			QueryPerformanceFrequency(&freq);
			
			rgb = cvQueryFrame(capture);
			cvCvtColor(rgb, f2, CV_BGR2GRAY);

			QueryPerformanceCounter(&start);
			
			cvCalcOpticalFlowPyrLK(f1, f2, pyr1, pyr2, corns1, corns2, cnt, 
				cvSize(win_size,win_size), 3, features_fnd, feature_errs, 
				cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3), 0);
			QueryPerformanceCounter(&stop);
			double time_us = 
				(stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
			printf("%g\n", time_us);

			cvClearSeq(wr.seq);
			cvStartAppendToSeq(wr.seq, &wr);
			for(int i=0; i <  cnt; i++){
				if(features_fnd[i] == 0 && feature_errs[i] > 550) {
					printf("Error is %f\n", feature_errs[i]);
					continue;
				}
				//printf("Got it dx: %g dy: %g err: %g\n", corns2[i].x-corns1[i].x,
				//	corns2[i].y-corns1[i].y, feature_errs[i]);
				CvPoint p0 = 
					cvPoint( cvRound( corns1[i].x ), cvRound( corns1[i].y ));
				CvPoint p1 = 
					cvPoint( cvRound( corns2[i].x ), cvRound( corns2[i].y ));
				cvLine( rgb2, p0, p1, CV_RGB(255,0,0), 2 );	
				cvCircle(rgb2, p1, 2, CV_RGB(0,255,255));

				CV_WRITE_SEQ_ELEM(p1, wr);
			}
			cvEndWriteSeq(&wr);

			float rd;
			CvPoint2D32f cxy;
			if(wr.seq->total && cvMinEnclosingCircle(wr.seq, &cxy, &rd)) {
				cvCircle(rgb2, cvPoint((int) cxy.x, (int) cxy.y), cvRound(rd), 
					CV_RGB(0,255,0));
				
				cvSetImageROI(gray, cvRect(cvRound(cxy.x - w/2), 
					cvRound(cxy.y - h/2), w, h));
			}
			else {
				printf("circle not found\n");
			}

			cvShowImage("f1", f1);
			cvShowImage("f2", f2);
			cvShowImage("rgb2", rgb2);
		}


#if 0
		// grab an image
		rgb = cvQueryFrame(capture);

		cvSetImageROI(rgb, cvRect(gray->roi->xOffset, gray->roi->yOffset,
			gray->roi->width, gray->roi->height));
		cvCvtColor(rgb, gray, CV_BGR2GRAY);

		position(gray, &wr, w, h, kal, z_k, u_k);
		draw_position(gray, rgb, wr.seq, kal);
#endif
	}

	w = 40;
	h = 40;
	x = 301;
	y = 172;

	cvSetImageROI(gray, cvRect(x-w/2, y-h/2, w, h));
	while(cvWaitKey(100) != 'q') {
		// grab an image
		break;
		rgb = cvQueryFrame(capture);

		cvSetImageROI(rgb, cvRect(gray->roi->xOffset, gray->roi->yOffset,
			gray->roi->width, gray->roi->height));
		cvCvtColor(rgb, gray, CV_BGR2GRAY);

		position(gray, &wr, w, h, kal, z_k, u_k);
		draw_position(gray, rgb, wr.seq, kal);
	}

	cvReleaseKalman(&kal);
	cvReleaseMemStorage(&mem);
	cvReleaseImage(&gray);
	cvReleaseCapture(&capture);
	return 0;
}






#if 0
int position(IplImage *gray, int w, int h, IplImage *rgb)
{
	double time_us;
	int x, y;
	LARGE_INTEGER start, stop, freq;
	CvMoments m;
	CvSeqWriter wr;


	IplImage *dup = cvCloneImage(gray);
	CvMemStorage *mem = cvCreateMemStorage(0);
	CvPoint pt;
	cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
		sizeof(CvSeq), sizeof(CvPoint), mem, &wr);
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&start);
	cvThreshold(gray, gray, 100, 255, CV_THRESH_BINARY_INV);
	cvMoments(gray, &m, 1);

	char *pxl = gray->imageData + gray->roi->yOffset*gray->widthStep + gray->roi->xOffset;
	for(x = 0; x < w; x++) {
		for(y = 0; y < h; y++) {
			// can speed up by halving h&w && only doing one comparison
			if((pxl[x + ((y-1) * gray->widthStep)] == 0 &&
				pxl[x + (y * gray->widthStep)] != 0 &&
				pxl[x + ((y+1) * gray->widthStep)] != 0) ||
				(pxl[x + ((y-1) * gray->widthStep)] != 0 &&
				pxl[x + (y * gray->widthStep)] != 0 &&
				pxl[x + ((y+1) * gray->widthStep)] == 0)) {

					pt.x = x;
					pt.y = y;
					CV_WRITE_SEQ_ELEM(pt, wr);


				(dup->imageData + 
					dup->roi->yOffset*gray->widthStep + 
					dup->roi->xOffset)[x + (y * gray->widthStep)] = 255;
			}
		}
	}

	if(m.m00 <= 0) {
		QueryPerformanceCounter(&stop);
		time_us = (stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
		printf("%g (%g)\n", time_us, time_us / (gray->width * gray->height));
		return !0;
	}


	float rad  = 1;
	CvPoint2D32f cen = {0,0};
	cvFlushSeqWriter(&wr);
	CvSeq *ptr_seq = cvEndWriteSeq(&wr);
	cvMinEnclosingCircle(ptr_seq, &cen, &rad);

	x = gray->roi->xOffset + cvRound(m.m10/m.m00 - w/2);
	y = gray->roi->yOffset + cvRound(m.m01/m.m00 - h/2);
	cvSetImageROI(gray, cvRect(x, y, w, h));

	QueryPerformanceCounter(&stop);

	cvCopyImage(dup, gray);

	cvCircle(rgb, cvPoint(cvRound(cen.x), cvRound(cen.y)), cvRound(rad), 
		CV_RGB(0,0,255), 3);

	time_us = (stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
	printf("%g (%g)\n", time_us, time_us / (gray->width * gray->height));
	printf("%g (%g, %g)\n", rad, cen.x, cen.y);

	return 0;
}

int main(int argc, char *argv[])
{
	// insert code
	CvCapture *capture = NULL;
	IplImage *rgb, *gray, *eig, *temp;
	CvMemStorage *storage;

	capture = cvCaptureFromCAM(0);
	if(!capture) {
		printf("init_cam_src: can't open camera\n");
		return -1;
	}

	rgb = cvQueryFrame(capture);
	gray = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	IplImage *canny = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_8U, 1);
	eig = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_32F, 1);
	temp = cvCreateImage(cvGetSize(rgb), IPL_DEPTH_32F, 1);
	storage = cvCreateMemStorage(0);
	IplImage *pyr = cvCreateImage(cvSize(rgb->width/2, rgb->height/2), 8, 1);
	CvPoint2D32f pos;
	float r;
	CvSeq *fc;

	int ncorners = rgb->width*rgb->height;

	LARGE_INTEGER freq, start, stop;
	QueryPerformanceFrequency(&freq);
	CvMoments mom;

	char text[100];
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,1,0,1);

	int w = 40;
	int h = 40;
	int x = 271-w/2;
	int y = 144-h/2;
	cvSetImageROI(gray, cvRect(x, y, w, h));

	while(cvWaitKey(100) != 'q') {
		rgb = cvQueryFrame(capture);

		cvSetImageROI(rgb, cvRect(gray->roi->xOffset, gray->roi->yOffset,
			gray->roi->width, gray->roi->height));
		cvCvtColor(rgb, gray, CV_BGR2GRAY);

		position(gray, w, h, rgb);

		cvResetImageROI(rgb);
		memset(text, 0, 100);
		sprintf_s(text, 100, "(%d,%d)", gray->roi->xOffset, gray->roi->yOffset);
		cvPutText(rgb, text, cvPoint(gray->roi->xOffset, gray->roi->yOffset), 
			&font, CV_RGB(0,255,0));

		cvShowImage("rgb", rgb);
		cvShowImage("gray", gray);
	}

/*
	while(cvWaitKey(100) != 'q') {
		rgb = cvQueryFrame(capture);

		cvCvtColor(rgb, gray, CV_BGR2GRAY);
		cvCanny(gray, canny, 50, 100);

		QueryPerformanceCounter(&start);
		cvPyrDown(gray, pyr, CV_GAUSSIAN_5x5);
		cvPyrUp(pyr, gray, CV_GAUSSIAN_5x5);

		cvCanny(gray, gray, 50, 100);

		cvFindContours(gray, storage, &fc);
		for(CvSeq *c = fc; c != NULL; c = c->h_next) {
			cvContourMoments(c, &mom);

			double e1 = (mom.mu20 + mom.mu02)/2 + 
				sqrt(4*pow(mom.mu11, 2) + pow(mom.mu20 - mom.mu02, 2))/2;
			double e2 = (mom.mu20 + mom.mu02)/2 - 
				sqrt(4*pow(mom.mu11, 2) + pow(mom.mu20 - mom.mu02, 2))/2;


			double e = sqrt(1 - e2/e1);

			double k = pow(cvContourPerimeter(c), 2) /(4*acos(-1.0)*cvContourArea(c));

			if(cvMinEnclosingCircle(c, &pos, &r) && r < 35) {
				printf("r: %g e: %g k: %g\n", r, e, k);
				if(abs(e) < .6) {
					cvDrawCircle(rgb, cvPoint(cvRound(pos.x), cvRound(pos.y)), 
						cvRound(r), CV_RGB(0,0,255));

					memset(text, 0, 100);
					sprintf_s(text, 100, "(%d,%d)", cvRound(pos.x), cvRound(pos.y));
					cvPutText(rgb, text, cvPoint(cvRound(pos.x), cvRound(pos.y)), 
						&font, CV_RGB(0,255,0));

				}
			}
		}

		QueryPerformanceCounter(&stop);

		cvDrawContours(rgb, fc, CV_RGB(0,255,0), CV_RGB(255,0,0), 3);
		double time_us = (stop.QuadPart - start.QuadPart) / (freq.QuadPart * 1.0) * 1e6;
		printf("%g (%g)\n", time_us, time_us / (rgb->width * rgb->height));

		cvShowImage("color", rgb);
		cvShowImage("gray", gray);
		cvShowImage("canny", canny);
	}
	*/

	cvReleaseCapture(&capture);
	return 0;
}
#endif