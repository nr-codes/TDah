#include "t_dah.h"
#include "TDahOpenCV.h"


int main()
{
	TDahOpenCV *capture = new TDahOpenCV(CV_CAP_ANY);
	ROILoc r;
	IplImage *img;

	capture->initROIs(1, 20, 20, "myopencv.yaml", false, true);
	img = cvQueryFrame(capture);
	img = cvCreateImage(cvSize(img->width, img->height), 8, 1);
	r.img = img;
	while(cvWaitKey(100) != 'q') {
		capture->showROILoc();
		capture->grabFrame();
		capture->getROILoc(3, &r);
		cvShowImage("img", img);
		printf("%d (%d, %d)\n", r.roi_nr, r.loc.x, r.loc.y);
	}

	delete capture;
	return 0;
}















#if 0
#define APP_NUM_ROI 2
#define MAX_RADIUS 10
#define MIN_MATCH 0.89
#define ROWS 2
#define COLS 1
// CAMERA REGION OF INTEREST
#define ROI_BOX 40
#define ROI_WIDTH ROI_BOX
#define ROI_HEIGHT ROI_BOX

static CvCapture *capture;

IplImage *grab_img()
{
	return 	cvQueryFrame(capture);
}

int main()
{
	int rc;

#if 1
	int ra;
	rc = init_system(APP_NUM_ROI, "myopencv.yaml");
	if(rc == CV_OK) {
		ROILoc roi[2];
			while(rc++ < 100) {
				ra = queryFrameOpenCV(0, &roi[rc%2]);
				//printf("%d\n", ra);
				//queryFrameOpenCV(0, &roi[1]);
			}
		for(int i = 0; i < APP_NUM_ROI; i++) {
			printf("%x (%d, %d) %g %d\n", 
				roi[i].roi_nr, roi[i].loc.x, roi[i].loc.y, roi[i].ts, roi[i].img_nr);
		}
	}
	rc = deinit_system();
#else	
	rc = init_system_opencv(CV_CAP_ANY, ROI_WIDTH, 
		ROI_HEIGHT, APP_NUM_ROI, "myopencv.yaml", 0);
	if(rc == CV_OK)
		rc = deinit_system();
#endif
	
	printf("return code: %d\n", rc);
	cvNamedWindow("hi");
	cvWaitKey();
	return rc;


	int state;
	int t = 0, j;
	double score;	
	IplImage *tplt[APP_NUM_ROI], *img, *gr[APP_NUM_ROI], *temp;
	CvKalman *kal[APP_NUM_ROI];
	CvSeqWriter wr[APP_NUM_ROI];
	CvMemStorage *mem[APP_NUM_ROI];
	CvPoint c;
	float z[Z_DIM];
	double dt, start, stop;

	// init everything
	capture = cvCaptureFromCAM(CV_CAP_ANY);
	img = cvQueryFrame(capture);
	for(int i = 0; i < APP_NUM_ROI; i++) {
		gr[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		tplt[i] = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		cvSetImageROI(gr[i], cvRect(0, 0, img->width, img->height));
		cvSetImageROI(tplt[i], cvRect(0, 0, img->width, img->height));

		kal[i] = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
		mem[i] = cvCreateMemStorage(0);
		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
							sizeof(CvSeq), sizeof(CvPoint), mem[i], &wr[i]);
	}
	temp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	cvZero(temp);

	setup_kalman(kal, APP_NUM_ROI);

	// get dots
	rc = manual_acquire(&grab_img, gr, ROI_WIDTH, ROI_HEIGHT, 
		&t, APP_NUM_ROI, tplt, kal);
	show_tplts(tplt, ROI_WIDTH, ROI_HEIGHT, ROWS, COLS, APP_NUM_ROI);
	cvWaitKey(0);
	cvDestroyAllWindows();

#if 0
	CvFileStorage *fs = cvOpenFileStorage("tdahconf.yaml", NULL, CV_STORAGE_WRITE);
	write_opencv_header(fs, CV_CAP_ANY);
	write_templates(fs, tplt, APP_NUM_ROI);
	write_track_params(fs, t, MIN_MATCH, MAX_RADIUS, ROI_WIDTH, ROI_HEIGHT,
		img->width, img->height);
	write_kalman(fs, kal, APP_NUM_ROI);
	write_obj_loc(fs, gr, APP_NUM_ROI);
	cvReleaseFileStorage(&fs);
#else
	CvFileStorage *fs = cvOpenFileStorage("tdahconf.yaml", NULL, CV_STORAGE_READ);
	const char *applet;
	double exposure, ft;
	int buffers;
	int index;
	read_opencv_header(fs, &index, &exposure);
	read_me3_header(fs, &applet, &exposure, &ft, &buffers);

	read_templates(fs, tplt, APP_NUM_ROI);
	show_tplts(tplt, ROI_WIDTH, ROI_HEIGHT, ROWS, COLS, APP_NUM_ROI);

	int thr, rw, rh, iw, ih;
	double mm, mr;
	read_track_params(fs, &thr, &mm, &mr, &rw, &rh, &iw, &ih);

	//printf("%s %g %g %d\n", applet, exposure, ft, buffers);
	printf("%d %d %g %g %d\n", rw, rh, mm, mr, thr);

	read_kalman(fs, kal, 2);

	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < X_DIM; j++) {
			printf("%f ", kal[i]->state_post->data.fl[j]);
		}

		printf("\n");
		printf("\n");
		for(int j = 0; j < X_DIM*X_DIM; j++) {
			printf("%f ", kal[i]->error_cov_post->data.fl[j]);
		}
		printf("\n");
		printf("\n");
	}

	read_obj_loc(fs, gr, APP_NUM_ROI);

	for(int i =0; i<2; i++) {
		CvRect roi = cvGetImageROI(gr[i]);
		printf("%d %d %d %d\n", roi.x, roi.y, roi.width, roi.height);
	}

	cvReleaseFileStorage(&fs);
#endif


	cvNamedWindow("hi");
	cvWaitKey();
	cvReleaseCapture(&capture);
	return 0;

	rc = auto_acquire(capture, gr, ROI_WIDTH, ROI_HEIGHT, t, MAX_RADIUS, 
		APP_NUM_ROI, tplt, MIN_MATCH, kal, 1);
	show_tplts(tplt, ROI_WIDTH, ROI_HEIGHT, ROWS, COLS, APP_NUM_ROI);
	cvWaitKey(0);
	cvDestroyAllWindows();


	cvNamedWindow("pyr", 0);
	j = 0;
	start = cvGetTickCount()/cvGetTickFrequency();
	while(cvWaitKey(100) != 'q') {
		img = cvQueryFrame(capture);

		stop = cvGetTickCount()/cvGetTickFrequency();
		dt = (stop - start) * 1e-6;
		start = cvGetTickCount()/cvGetTickFrequency();

		cvSetImageROI(img, cvGetImageROI(gr[j]));
		cvCvtColor(img, gr[j], CV_BGR2GRAY);

		state = 0;
		score = track_ctrd(gr[j], ROI_WIDTH, ROI_HEIGHT, t, &wr[j]);
		if(!score || score > MAX_RADIUS) {
			cvResetImageROI(gr[j]);
			cvResetImageROI(img);
			cvCvtColor(img, gr[j], CV_BGR2GRAY);
			state = 1;
			score = track_tmplt_pyr(gr[j], tplt[j], temp, 2, 5);

			if(score < MIN_MATCH) {
				cvResetImageROI(gr[j]);
				state = 2;
				score = track_tmplt(gr[j], tplt[j]);
				if(score < MIN_MATCH) {
					state = 3;
				}
			}
		}

		c = roi2ctrd(gr[j]);
		z[0] = (float) c.x;
		z[1] = (float) c.y;
		prediction(kal[j], (float) dt, z);
		ctrd2roi(gr[j], 
					cvRound(kal[j]->state_post->data.fl[0]), 
					cvRound(kal[j]->state_post->data.fl[1]),
					ROI_WIDTH, 
					ROI_HEIGHT);

		cvResetImageROI(img);
		show_position(gr, APP_NUM_ROI, kal, wr, NULL, img);
		show_seqs(wr, ROI_WIDTH, ROI_HEIGHT, ROWS, COLS, APP_NUM_ROI);

		cvResetImageROI(temp);
		cvShowImage("pyr", temp);

		if(state == 0) {
			printf("%d: centroid -- %0.4g\n", j, score);
		}
		else if(state == 1) {
			printf("%d: pyramid template -- %0.4g\n", j, score);
		}
		else if (state == 2) {
			printf("%d: full template -- %0.4g\n", j, score);
		}
		else if(state == 3) {
			printf("%d: object lost -- %0.4g\n", j, score);
		}


		j++;
		j %= APP_NUM_ROI;
	}

	cvReleaseCapture(&capture);
	return 0;
}
#endif