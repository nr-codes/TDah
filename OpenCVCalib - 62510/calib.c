// image open code taken from 
// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00025000000000000000

// grid orientation
//http://opencv-users.1802565.n2.nabble.com/FindChessboardCorners-ordering-td2122706.html

#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <conio.h>
#include <errno.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#define IMAGE "imgs/Image4.tif"
#define SQ_WIDTH 11
#define SQ_HEIGHT 12

void PrintMat(CvMat *A);

int main()
{
	int rc;
	IplImage *img = NULL, *gray_img = NULL;
	CvSize size, window;
	CvPoint2D32f corners[SQ_WIDTH*SQ_HEIGHT];
	int corners_found = 0;
	CvMat* image_points	= cvCreateMat( SQ_WIDTH*SQ_HEIGHT, 2, CV_32FC1 );
	CvMat* object_points = cvCreateMat( SQ_WIDTH*SQ_HEIGHT, 3, CV_32FC1 );
	CvMat* point_counts = cvCreateMat( 1, 1, CV_32SC1 );
	CvMat* intrinsic_matrix = cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs = cvCreateMat( 4, 1, CV_32FC1 );

	img = cvLoadImage(IMAGE, CV_LOAD_IMAGE_COLOR /*CV_LOAD_IMAGE_GRAYSCALE*/);
	if(img == NULL) {
		return -1;
	}

	gray_img = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U , 1);


	// create a window
	cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE); 
	cvMoveWindow("mainWin", 100, 100);
	
	size = cvSize(SQ_WIDTH, SQ_HEIGHT);
	rc = cvFindChessboardCorners(img, size, corners, &corners_found, 
		CV_CALIB_CB_FILTER_QUADS);

	cvCvtColor(img, gray_img, CV_BGR2GRAY);

	window = cvSize(5, 5);
	cvFindCornerSubPix(gray_img, corners, corners_found, window, cvSize( -1, -1 ), 
		cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
	
	cvDrawChessboardCorners(img, size, corners, corners_found, rc);

	// show the image
	for(rc = 0; rc < SQ_WIDTH; rc++) {
		cvRectangle(img, cvPoint(corners[rc].x-10, corners[rc].y-10), 
			cvPoint(corners[rc].x+10, corners[rc].y+10), 
			cvScalar(128,3*rc,rc*15,0), 1, 8, 0);
	}
	cvShowImage("mainWin", img);

	for(rc = 0; rc < corners_found; rc++) {
		printf("%d %f %f\n", rc, corners[rc].x, corners[rc].y);
	}

	for(rc = 0; rc < corners_found; rc++){
		CV_MAT_ELEM( *image_points, float, rc, 0 ) = corners[rc].x;
		CV_MAT_ELEM( *image_points, float, rc, 1 ) = corners[rc].y;
		CV_MAT_ELEM( *object_points, float, rc, 0 ) = 30*(rc%30);
		CV_MAT_ELEM( *object_points, float, rc, 1 ) = (int) rc/30;
		CV_MAT_ELEM( *object_points, float, rc, 2 ) = 0.0f;
	}
	CV_MAT_ELEM( *point_counts, int, 0, 0 ) = corners_found;
	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	cvCalibrateCamera2(object_points, image_points, point_counts, cvGetSize(img), 
		intrinsic_matrix, distortion_coeffs, 0, 0, 0);

	PrintMat(intrinsic_matrix);
	PrintMat(distortion_coeffs);

	// wait for a key
	cvWaitKey(0);

	// release the image
	cvReleaseImage(&img);
	return 0;
}

//http://blog.weisu.org/2007/11/opencv-print-matrix.html
void PrintMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++)
	{
	printf("\n"); 
	switch (CV_MAT_DEPTH(A->type))
	{
	case CV_32F:
	case CV_64F:
	for (j = 0; j < A->cols; j++)
	printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
	break;
	case CV_8U:
	case CV_16U:
	for(j = 0; j < A->cols; j++)
	printf ("%6d",(int)cvGetReal2D(A, i, j));
	break;
	default:
	break;
	}
	}
	printf("\n");
}