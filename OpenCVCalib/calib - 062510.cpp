// image open code taken from 
// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00025000000000000000
// last accessed 06/24/10

// grid orientation
//http://opencv-users.1802565.n2.nabble.com/FindChessboardCorners-ordering-td2122706.html
// last accessed 06/24/10

#include <fcdynamic.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#define CALIBRATE 1

#define MAIN "mainWin"
#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define EXPOSURE 20000
#define FRAME_TIME 50000
#define TIMEOUT 5
#define SEQ_LEN 1

#define SQ_WIDTH 6
#define SQ_HEIGHT 3
#define SQ_MM 34
#define NUM_IMAGES 100
#define NUM_POINTS (SQ_WIDTH*SQ_HEIGHT)

void PrintMat(CvMat *A);
void init_roi0(TrackingWindow *win, int roi_box, double frame, double exposure);
void copygray2rgb(IplImage *chan1Img, IplImage *chan3Img);

int main()
{
	int rc, img_nr, input;
	Fg_Struct *fg = NULL;
	TrackingSequence tseq;
	
	IplImage *img = NULL, *output = NULL;
	CvSize size;
	CvPoint2D32f corners[NUM_POINTS];
	CvMat *image_points, *object_points, *intrinsic_matrix, *point_counts, *distortion_coeffs;
	int corners_found = 0, good_imgs = 0;

	img = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U , 1);
	output = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U , 3);
	if(img == NULL || output == NULL) {
		return -1;
	}

	tseq.seq = new int[SEQ_LEN];
	tseq.seq[0] = ROI_0;
	tseq.seq_len = SEQ_LEN;
	init_roi0(tseq.windows, IMG_WIDTH, FRAME_TIME, EXPOSURE);
	rc = StartGrabbing(&fg, &tseq, NULL);
	if(rc != FG_OK) {
		return -2;
	}

	// create a window
	cvNamedWindow(MAIN, CV_WINDOW_AUTOSIZE);

	// allocate calibration data
	size = cvSize(SQ_WIDTH, SQ_HEIGHT);
	image_points = cvCreateMat(NUM_IMAGES*NUM_POINTS, 2, CV_32FC1);
	object_points = cvCreateMat(NUM_IMAGES*NUM_POINTS, 3, CV_32FC1);
	point_counts = cvCreateMat(NUM_IMAGES, 1, CV_32SC1);
	
	img_nr = 1;
#if CALIBRATE
	while(good_imgs < NUM_IMAGES) {
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		tseq.windows[ROI_0].img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		if(tseq.windows[ROI_0].img != NULL) {
			CopyTrackingWindowToImage(tseq.windows, img);

			rc = cvFindChessboardCorners(img, size, corners, &corners_found);
			if(rc) {
				cvFindCornerSubPix(img, corners, corners_found, cvSize(5, 5), cvSize( -1, -1 ), 
					cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
				
				//store pixel points
				CV_MAT_ELEM(*point_counts, int, good_imgs, 0) = corners_found;
				for(int i = 0; i < NUM_POINTS; i++) {
					CV_MAT_ELEM(*image_points, float, good_imgs*NUM_POINTS + i, 0) = corners[i].x;
					CV_MAT_ELEM(*image_points, float, good_imgs*NUM_POINTS + i, 1) = corners[i].y;
					CV_MAT_ELEM(*object_points, float, good_imgs*NUM_POINTS + i, 0) = (float) (SQ_MM*(i%SQ_WIDTH));
					CV_MAT_ELEM(*object_points, float, good_imgs*NUM_POINTS + i, 1) = (float) (((int) i/SQ_WIDTH)*SQ_MM);
					CV_MAT_ELEM(*object_points, float, good_imgs*NUM_POINTS + i, 2) = 1.0f;
				}
				good_imgs++;
			}
			copygray2rgb(img, output);
			cvDrawChessboardCorners(output, size, corners, corners_found, rc);
			cvShowImage(MAIN, output);

			
			// get input
			input = cvWaitKey(1);
			if(input == 'q') {
				break;
			}
		}
		else {
			printf("img is null\n");
			break;
		}
	}

	distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);
	intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1219.49486f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 1) = 0;
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 2) = 502.28854f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 0) = 0;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1227.09415f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 2) = 524.25824f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 2, 0) = 0;
	CV_MAT_ELEM(*intrinsic_matrix, float, 2, 1) = 0;
	CV_MAT_ELEM(*intrinsic_matrix, float, 2, 2) = 1;

	cvCalibrateCamera2(object_points, image_points, point_counts, cvGetSize(img), 
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_USE_INTRINSIC_GUESS);

	PrintMat(intrinsic_matrix);
	PrintMat(distortion_coeffs);

	// Save the intrinsics and distortions
	cvSave( "Intrinsics.xml", intrinsic_matrix );
	cvSave( "Distortion.xml", distortion_coeffs );

#endif

	// load these matrices back in
	CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );
	CvMat *world_Rvec = cvCreateMat(3, 1, CV_32FC1);
	CvMat *world_R = cvCreateMat(3, 3, CV_32FC1);
	CvMat *world_T = cvCreateMat(3, 1, CV_32FC1);

	// world coordinates
	CvMat *world_image_points = cvCreateMat(NUM_POINTS, 2, CV_32FC1);
	CvMat *world_object_points = cvCreateMat(NUM_POINTS, 3, CV_32FC1);
	CvMat *world_point_counts = cvCreateMat(1, 1, CV_32SC1);

	while(1) {
		input = cvWaitKey(0);
		if(input == 'q') {
			break;
		}

		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		tseq.windows[ROI_0].img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		if(tseq.windows[ROI_0].img != NULL) {
			CopyTrackingWindowToImage(tseq.windows, img);

			rc = cvFindChessboardCorners(img, size, corners, &corners_found);
			if(rc) {
				cvFindCornerSubPix(img, corners, corners_found, cvSize(5, 5), cvSize( -1, -1 ), 
					cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
				
				//store pixel points
				CV_MAT_ELEM(*world_point_counts, int, 0, 0) = corners_found;
				for(int i = 0; i < NUM_POINTS; i++) {
					CV_MAT_ELEM(*world_image_points, float, i, 0) = corners[i].x;
					CV_MAT_ELEM(*world_image_points, float, i, 1) = corners[i].y;
					CV_MAT_ELEM(*world_object_points, float, i, 0) = (float) (SQ_MM*(i%SQ_WIDTH));
					CV_MAT_ELEM(*world_object_points, float, i, 1) = (float) (((int) i/SQ_WIDTH)*SQ_MM);
					CV_MAT_ELEM(*world_object_points, float, i, 2) = 1.0f;
				}
			}
			copygray2rgb(img, output);
			cvDrawChessboardCorners(output, size, corners, corners_found, rc);
			cvShowImage(MAIN, output);
		}
		else {
			printf("img is null\n");
			return 0;
		}
	}

	// undistortion code
	IplImage *mapx = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	IplImage *mapy = cvCreateImage( cvGetSize( img ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );
	cvNamedWindow( "Undistort" );

	cvFindExtrinsicCameraParams2(world_object_points, world_image_points, intrinsic, distortion, world_Rvec, world_T);
	CvMat *normalized = cvCreateMat(1, 1, CV_32FC2);
	CvMat *distorted = cvCreateMat(1, 1, CV_32FC2);
	CvMat *world = cvCreateMat(3, 1, CV_32FC1);
	float fx = CV_MAT_ELEM(*intrinsic, float, 0, 0);
	float cx = CV_MAT_ELEM(*intrinsic, float, 0, 2);
	float fy = CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float cy = CV_MAT_ELEM(*intrinsic, float, 1, 2);
	double Z = 1358.9;
	cvRodrigues2(world_Rvec, world_R);
	for(int i = 0; i < NUM_POINTS; i++) {
		double u = corners[i].x;
		double v = corners[i].y;
		double xp = (u - cx)/fx;
		double yp = (v - cy)/fy;

		//CV_MAT_ELEM(*distorted, float, 0, 0) = (float) xp;
		//CV_MAT_ELEM(*distorted, float, 1, 0) = (float) yp;
		//((double*)(mat->data.ptr + mat->step*i))[j*2]
		((float *)(distortion->data.ptr))[0] = (float) xp;
		((float *)(distortion->data.ptr))[1] = (float) yp;

		cvUndistortPoints(distorted, normalized, intrinsic, distortion, NULL, intrinsic);
		CV_MAT_ELEM(*world, float, 0, 0) = ((float *)(normalized->data.ptr))[0];//(float) (CV_MAT_ELEM(*normalized, float, 0, 0)*Z);
		CV_MAT_ELEM(*world, float, 1, 0) = ((float *)(normalized->data.ptr))[1];//(float) (CV_MAT_ELEM(*normalized, float, 1, 0)*Z);
		CV_MAT_ELEM(*world, float, 2, 0) = (float) 1;
		

		//cvSub(world, world_T, world);
		cvGEMM(world_R, world, 1, NULL, 0, world, 0*CV_GEMM_A_T);

		printf("%d %f %f %f\n", i, CV_MAT_ELEM(*world, float, 0, 0), 
			CV_MAT_ELEM(*world, float, 1, 0), CV_MAT_ELEM(*world, float, 2, 0));
	}


	while(1) {
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		tseq.windows[ROI_0].img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);

		if(tseq.windows[ROI_0].img != NULL) {
			CopyTrackingWindowToImage(tseq.windows, img);
			IplImage *t = cvCloneImage(img);
			cvShowImage(MAIN, img); // Show raw image
			cvRemap(t, img, mapx, mapy); // undistort image
			cvReleaseImage(&t);
			cvShowImage( "Undistort", img); // Show corrected image
		}
		else {
			printf("img is null\n");
			break;
		}

		// get input
		input = cvWaitKey(1);
		if(input == 'q') {
			break;
		}
	}

	rc = deinit_cam(fg);
	if(rc != FG_OK) {
		printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
		return rc;
	}
	cvReleaseImage(&img);
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
	cvReleaseImage(&output);
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);

#if CALIBRATION
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
#endif

	return 0;
}

//code taken from: http://blog.weisu.org/2007/11/opencv-print-matrix.html
// last accessed 06/24/10
void PrintMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++) {
		printf("\n");
		switch (CV_MAT_DEPTH(A->type)) {
			case CV_32F:
			case CV_64F:
				for (j = 0; j < A->cols; j++) {
					printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
				}
				break;
			case CV_8U:
			case CV_16U:
				for(j = 0; j < A->cols; j++) {
					printf ("%6d",(int)cvGetReal2D(A, i, j));
				}
				break;
			default:
				break;
		}
	}
	printf("\n");
}

void copygray2rgb(IplImage *chan1Img, IplImage *chan3Img) 
{
	// copy from grayscale to RGB for color in cvDrawChessboardCorners
	for(int i = 0; i < IMG_WIDTH*IMG_HEIGHT; i++) {
		chan3Img->imageData[3*i] = chan1Img->imageData[i];
		chan3Img->imageData[3*i + 1] = chan1Img->imageData[i];
		chan3Img->imageData[3*i + 2] = chan1Img->imageData[i];
	}
}

void init_roi0(TrackingWindow *win, int roi_box, double frame, double exposure)
{
	memset(win, 0, sizeof(TrackingWindow) * MAX_ROI);
	win[ROI_0].blob_xmin = 0;
	win[ROI_0].blob_ymin = 0;
	win[ROI_0].blob_xmax = IMG_WIDTH;
	win[ROI_0].blob_ymax = IMG_HEIGHT;
	win[ROI_0].roi = ROI_0;
	win[ROI_0].roi_w = roi_box;
	win[ROI_0].roi_h = roi_box;
	win[ROI_0].img_w = IMG_WIDTH;
	win[ROI_0].img_h = IMG_HEIGHT;
	set_roi_box(win, IMG_WIDTH/2, IMG_HEIGHT/2);
	fix_blob_bounds(win);
	SetTrackCamParameters(win, frame, exposure);
}
