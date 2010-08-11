#include "t_dah.h"
#include "TDahMe3Fc.h"
#include <conio.h>

// me3 parametesr
#define TRIG GRABBER_CONTROLLED
#define EXPOSURE 450 // us
#define FRAME 550 // us
#define BUFS 100

// calib parameters
#define NONE 0
#define INTRINSIC 1
#define EXTRINSIC 2
#define CALIB 0

#define INTRINS_FILE "TrackCam Intrinsics.yaml"
#define EXTRINS_FILE "TrackCam Extrinsics.yaml"

#define GRID_W 3
#define GRID_H 6
#define INCHES_PER_SQR 1.33858268
#define NUM_IMGS 16

// ROI parameters
#define NUM_ROI 1
#define ROI_W 36
#define ROI_H 36

#define HAVE_CONF 0
#define CONF "myme3.yaml"
#define USE_KAL false
#define USE_TPT false

#define GR_CH 1


#define NIMGS 5000
IplImage *bimgs[NIMGS];
#define TXT_SIZE 50

int track_dots(TDahMe3Fc *capture)
{
	int i;
	ROILoc r;

	if(HAVE_CONF) {
		// a config file exists, use it to auto-acquire dots
		i = capture->initROIs(NUM_ROI, CONF, USE_KAL, USE_TPT);
	}
	else {
		// explicitly give ROI info and save config for later use
		i = capture->initROIs(NUM_ROI, ROI_W, ROI_H, CONF, USE_KAL, USE_TPT);
			//INTRINS_FILE, EXTRINS_FILE); DELETE
	}

	if(i != CV_OK) {
		return i;
	}

	i = 0;

	//cvNamedWindow("obj 0", 0);
	//cvNamedWindow("obj 1", 0);
	//cvNamedWindow("obj 2", 0);
	//cvNamedWindow("obj img", 0);
	//cvResizeWindow("obj img", ROI_W, ROI_H);

	r.img = cvCreateImage(cvSize(1024,1024), 8, 1);
	IplImage *img = cvCreateImage(cvSize(1024,1024), 8, 3);
	if(!r.img) return !CV_OK;

	int imgs_lost = 0, prev_img = 0;
	char text[TXT_SIZE];
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.5,0.5,0,1);

	while(1) {
		if(_kbhit()) break;

		// get and process next image
		capture->grabFrame();
		//++i; DELETE
		i+=2;
		prev_img = capture->getROILoc(i, &r);

		if(prev_img != i) imgs_lost++;
		
		if(r.img->roi) {
			cvCopyImage(r.img, bimgs[(i-1)/2 % NIMGS]);
			memset(text, 0, TXT_SIZE);	sprintf_s(text, TXT_SIZE, "%d,%d", r.loc.x, r.loc.y);
			cvPutText(bimgs[(i-1)/2 % NIMGS], text, cvPoint(0, 10), &font, CV_RGB(128,128,128));
			/*
			cvSetImageROI(img, cvGetImageROI(r.img));
			cvCvtColor(r.img, img, CV_GRAY2BGR);
			cvResetImageROI(img);
			draw_ctrd(img, r.img, NULL, 0); 
			cvSetImageROI(img, cvGetImageROI(r.img));
			cvCopyImage(img, bimgs[(i-1) % NIMGS]);
			cvWaitKey(1);
			*/
		}

		//cvResetImageROI(r.img);
		//cvWriteFrame(writer, r.img);

		//cvShowImage("img", r.img);
		//if(r.img->roi != NULL)
		//printf("%d (%d, %d) %d %d %d %d\n", r.roi_nr, r.loc.x, r.loc.y, r.img->roi->xOffset, r.img->roi->yOffset, r.img->roi->width, r.img->roi->height);
		//cvWaitKey(1);

		// visualize tracking
		//capture->showROILoc();
		//memset(text, 0, TXT_SIZE);	sprintf_s(text, TXT_SIZE, "%d", r.img_nr);
		//cvPutText(bimgs[(i-1) % NIMGS], text, cvPoint(0, ROI_H/2), &font, CV_RGB(128,128,128));
		//if(!r.obj_found || imgs_lost) {
		if(!r.obj_found) {
			printf("object lost (%d): (%d) %d (%d, %d) box: %d %d %d %d\n", r.obj_found, r.img_nr, r.roi_nr, r.loc.x, r.loc.y, 
				 r.img->roi->xOffset, r.img->roi->yOffset, r.img->roi->width, r.img->roi->height);
			printf("images lost: %d\n", imgs_lost);

			cvNamedWindow("small", 0);
			cvShowImage("small", r.img);
			
			cvResetImageROI(r.img);
			cvNamedWindow("big");
			cvShowImage("big", r.img);

			capture->showROILoc();
			cvWaitKey(0);


			prev_img = i;
			for(i = ((prev_img - 1)/2 + 1) % NIMGS; i < NIMGS; i++) {
				cvCopyImage(bimgs[((prev_img - 1)/2 % NIMGS)], bimgs[i]);
				memset(text, 0, TXT_SIZE);	sprintf_s(text, TXT_SIZE, "lost:");
				cvPutText(bimgs[i], text, cvPoint(0, ROI_H/4), &font, CV_RGB(255,255,255));
				/*
				memset(text, 0, TXT_SIZE);	sprintf_s(text, TXT_SIZE, "%d", r.img_nr);
				cvPutText(bimgs[i], text, cvPoint(0, ROI_H/2), &font, CV_RGB(255,255,255));
				memset(text, 0, TXT_SIZE);	sprintf_s(text, TXT_SIZE, "%d,%d", r.loc.x, r.loc.y);
				cvPutText(bimgs[i], text, cvPoint(0, 3*ROI_H/4), &font, CV_RGB(255,255,255));
				*/
			}
			break;
			//IplImage *img = cvQueryFrame(capture);
			//cvShowImage("w", img);
		}
	}

	CvVideoWriter *writer = NULL;
	writer = cvCreateVideoWriter("output.avi", CV_FOURCC('D', 'I', 'B', ' ') , 1e6 / FRAME / 10.,	cvSize(ROI_W,ROI_H), FALSE);

	if(!writer) {
		return !CV_OK;
	}

	for(i = 0; i < NIMGS; i++) {
		cvWriteFrame(writer, bimgs[i]);
	}
	cvReleaseVideoWriter(&writer);
	cvReleaseImage(&r.img);
	
	return CV_OK;
}

int main()
{
	int rc;
	TDahMe3Fc *capture = new TDahMe3Fc(TRIG, EXPOSURE, FRAME, BUFS);

	if(CALIB == INTRINSIC) {
		rc = get_camera_intrinsics(capture, INTRINS_FILE, 
			GRID_W, GRID_H, NUM_IMGS);
	}
	else if(CALIB == EXTRINSIC) {
		rc = get_camera_extrinsics(capture, EXTRINS_FILE, INTRINS_FILE, 
			GRID_W, GRID_H, INCHES_PER_SQR);
	}
	else {
		for(int i = 0; i < NIMGS; i++) {
			bimgs[i] = cvCreateImage(cvSize(ROI_W,ROI_H), 8, 1);
			if(!bimgs[i]) return !CV_OK;
			cvSet(bimgs[i], cvScalarAll(128));
		}
		rc = track_dots(capture);

		for(int i = 0; i < NIMGS; i++) {
			cvReleaseImage(&bimgs[i]);
		}
	}

	//_getch();
	
	delete capture;

	printf("press any key to quit\n");
	_getch();
	return rc;
}