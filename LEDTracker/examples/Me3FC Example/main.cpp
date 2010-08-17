#include "t_dah.h"
#include "TDahMe3Fc.h"
#include <conio.h>

// me3 parametesr
#define TRIG GRABBER_CONTROLLED 
#define EXPOSURE 50 // us
#define FRAME 200 // us
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
#define ROI_W 20
#define ROI_H 20

#define HAVE_CONF 0
#define CONF "myme3.yaml"
#define USE_KAL false
#define USE_TPT false

#define GR_CH 1


#define NIMGS 800
IplImage *bimgs[NIMGS];
ROILoc rr[NIMGS];
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

	r.img = cvCreateImage(cvSize(1024,1024), 8, 1);
	IplImage *img = cvCreateImage(cvSize(1024,1024), 8, 3);
	if(!r.img) return !CV_OK;

	int imgs_lost = 0, prev_img = 0;
	char text[TXT_SIZE];
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,0.5,0.5,0,1);

	int toggle = 0;
	while(1) {
		if(_kbhit() || -i == 2*NIMGS) break;

		// get and process next image
		capture->grabFrame();
		//++i; DELETE
		i+=2;
		prev_img = capture->getROILoc(i, &r);

		if(prev_img != i) imgs_lost++;
		
		if(r.img->roi) {
			cvCopyImage(r.img, bimgs[(i-1)/2 % NIMGS]);
			rr[(i-1)/2 % NIMGS] = r;
		}

		// output a ready signal
		toggle = 2 - toggle;
		capture->set_dio0(toggle);

		// visualize tracking
		//capture->showROILoc();
#if 1
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
			}
			break;
		}
#endif
	}

	if(i < 0) return !CV_OK;

	CvVideoWriter *writer = NULL;
	writer = cvCreateVideoWriter("output.avi", CV_FOURCC('D', 'I', 'B', ' ') , 1e6 / FRAME / 1000.,	cvSize(640,480));
	//writer = cvCreateVideoWriter("output.avi", -1, 1e6 / FRAME,	cvSize(1024,768));

	if(!writer) {
		return !CV_OK;
	}

	cvReleaseImage(&img);
	img = cvCreateImage(cvSize(640,480), 8, 3);

	for(i = 0; i < NIMGS; i++) {
		cvZero(img);
		cvSetImageROI(r.img, ctrd2roi(rr[i].loc.x, rr[i].loc.y, ROI_W, ROI_H));
		cvSetImageROI(img, ctrd2roi(rr[i].loc.x, rr[i].loc.y, ROI_W, ROI_H));

		printf("%d (%d,%d)\n", i, rr[i].loc.x, rr[i].loc.y);
		cvCvtColor(bimgs[i], img, CV_GRAY2BGR);
		cvResetImageROI(img);
		draw_ctrd(img, r.img, NULL, i % NUM_ROI);
		cvResetImageROI(r.img);

		cvWriteFrame(writer, img);
		cvShowImage("img", img);
		cvWaitKey(1);

		if(_kbhit() && _getch() == 'q') break;
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