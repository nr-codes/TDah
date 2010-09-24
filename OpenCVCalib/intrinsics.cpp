#include <fcdynamic.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "grab.h"

#define MAIN "mainWin"
#define SOURCE WEBCAM

#define SQ_WIDTH 6
#define SQ_HEIGHT 3
#define SQ_MM 34
#define NUM_IMAGES 50
#define NUM_POINTS (SQ_WIDTH*SQ_HEIGHT)

int main()
{
	int rc;
	IplImage *img = NULL;
	CvSize size;
	CvMat *image_points, *object_points, *intrinsic_matrix, 
		*point_counts, *distortion_coeffs;
	CvMat *rvecs, *tvecs, *per_view_errors;

	// allocate calibration data
	size = cvSize(SQ_WIDTH, SQ_HEIGHT);
	image_points = cvCreateMat(NUM_IMAGES*NUM_POINTS, 2, CV_32FC1);
	object_points = cvCreateMat(NUM_IMAGES*NUM_POINTS, 3, CV_32FC1);
	point_counts = cvCreateMat(NUM_IMAGES, 1, CV_32SC1);

	rvecs = cvCreateMat(NUM_IMAGES, 3, CV_32FC1);
	tvecs = cvCreateMat(NUM_IMAGES, 3, CV_32FC1);
	per_view_errors = cvCreateMat(NUM_IMAGES, 1, CV_64FC1);


	// grab an image so we know the image size
	init_cam_src(SOURCE);
	img = grab(SOURCE);
	printf("img_w: %d img_h: %d\n", img->width, img->height);
	CvSize img_size = cvGetSize(img);
	close_cam_src(SOURCE);


	rc = grab4calib(SOURCE, object_points, image_points, 
		point_counts, &size, NUM_IMAGES);
	if(rc != NUM_IMAGES) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, NUM_IMAGES);
		return -rc;
	}

	distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);
	intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	cvZero( intrinsic_matrix );
	cvZero( distortion_coeffs );

	cvCalibrateCamera2(object_points, image_points, point_counts, img_size, 
		intrinsic_matrix, distortion_coeffs, rvecs, tvecs);

	PrintMat(intrinsic_matrix);
	PrintMat(distortion_coeffs);

	double reproj_error = compute_reprojection_error(object_points,
        rvecs, tvecs, intrinsic_matrix, distortion_coeffs,
        image_points, point_counts, per_view_errors);

	printf("reprojecting error: %g\n", reproj_error);

	// Save the intrinsics and distortions
	cvSave("Intrinsics.xml", intrinsic_matrix );
	cvSave("Distortion.xml", distortion_coeffs );

	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);

	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);

	return 0;
}