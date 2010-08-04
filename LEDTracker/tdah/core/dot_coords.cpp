#include <errno.h>
#include "t_dah.h"

#define WIN_SIZE cvSize(5, 5)
#define ZERO_ZNE cvSize(-1, -1)
#define ERR_TOL cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1)
#define ONE_EXTRINSIC_IMG 1
#define PROMPT 1

static CvMat *A, *k, *R, *T;
static CvMat *world, *distorted, *normalized;

int setup_world_frame(char *A_file, char *k_file, char *R_file, char *T_file)
{
	// load camera model
	A = (CvMat *) cvLoad(A_file);
	if(A == NULL) return -EBADF;

	k = (CvMat *) cvLoad(k_file);
	if(k == NULL) return -EBADF;

	R = (CvMat *) cvLoad(R_file);
	if(R == NULL) return -EBADF;

	T = (CvMat *) cvLoad(T_file);
	if(T == NULL) return -EBADF;

	world = cvCreateMat(3, 1, CV_32FC1);
	if(world == NULL) return -ENOMEM;

	normalized = cvCreateMat(1, 1, CV_32FC2);
	if(normalized == NULL) return -ENOMEM;

	distorted = cvCreateMat(1, 1, CV_32FC2);
	if(distorted == NULL) return -ENOMEM;

	return CV_OK;
}

CvPoint2D32f pixel2world(CvPoint p, float z)
{
	CvPoint2D32f p_w;

	// convert from pixels to units of measurement
	distorted->data.fl[0] = (float) p.x;
	distorted->data.fl[1] = (float) p.y;
	cvUndistortPoints(distorted, normalized, A, k);

	cvmSet(world, 0, 0, z*normalized->data.fl[0]);
	cvmSet(world, 1, 0, z*normalized->data.fl[1]);
	cvmSet(world, 2, 0, z);

	cvSub(world, T, world);
	cvGEMM(R, world, 1, NULL, 0, world, CV_GEMM_A_T);

	p_w.x = world->data.fl[0];
	p_w.y = world->data.fl[1];

	return p_w;
}

int grab_calib_grid(CvCapture *capture, CvSize grid_size, 
					CvMat *ip, CvMat *op, CvMat *pc, 
					int num_imgs, int prompt, IplImage *store_imgs[])
{
	int rc, kb_input, rows, cols, num_points, corners_found, good_imgs, row0;
	IplImage *img, *gray;
	CvPoint2D32f *corners;

	good_imgs = 0;
	corners_found = 0;
	rows = grid_size.height;
	cols = grid_size.width;
	num_points = rows * cols;

	img = cvQueryFrame(capture);
	if(img == NULL) return good_imgs;
	gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	if(gray == NULL) return -ENOMEM;
	corners = (CvPoint2D32f *) cvAlloc(sizeof(CvPoint2D32f) * num_points);
	if(corners == NULL) {
		cvReleaseImage(&gray);
		return -ENOMEM;
	}

	cvNamedWindow("calibration", CV_WINDOW_AUTOSIZE);
	while(good_imgs < num_imgs) {
		img = cvQueryFrame(capture);
		if(img == NULL) break;
		cvCvtColor(img, gray, CV_BGR2GRAY);

		rc = cvFindChessboardCorners(gray, grid_size, corners, &corners_found);
		if(rc) {
			cvFindCornerSubPix(gray, corners, corners_found,
				WIN_SIZE, ZERO_ZNE, ERR_TOL);
			
			//store pixel points
			row0 = good_imgs*num_points;
			CV_MAT_ELEM(*pc, int, good_imgs, 0) = corners_found;
			for(int i = 0; i < num_points; i++) {
				cvmSet(ip, row0 + i, 0, corners[i].x);
				cvmSet(ip, row0 + i, 1, corners[i].y);

				if(op != NULL) {
					cvmSet(op, row0 + i, 0, i / cols);
					cvmSet(op, row0 + i, 1, i % cols);
					cvmSet(op, row0 + i, 2, 0);
				}
			}

			if(store_imgs != NULL) store_imgs[good_imgs] = cvCloneImage(img);
			good_imgs++;
		}

		cvDrawChessboardCorners(img, grid_size, corners, corners_found, rc);
		cvShowImage("calibration", img);

		if(rc && prompt) {
			// prompt user to keep results or overwrite it
			kb_input = cvWaitKey(0);
			if(kb_input == 'i') {
				good_imgs--;
			}
		}
		else {
			kb_input = cvWaitKey(1);
			if(kb_input == 'q') {
				break;
			}
		}
	}

	cvReleaseImage(&gray);
	cvFree(&corners);

	return good_imgs;
}

int get_camera_intrinsics(CvCapture *capture, int rows, int cols, int num_images)
{
	int rc, num_points;
	IplImage *img = NULL;
	CvSize size, img_size;
	CvMat *image_points, *object_points, *point_counts;
	CvMat *intrinsic_matrix, *distortion_coeffs;
	
	// allocate calibration data
	num_points = rows * cols;
	size = cvSize(cols, rows);

	image_points = cvCreateMat(num_images*num_points, 2, CV_32FC1);
	object_points = cvCreateMat(num_images*num_points, 3, CV_32FC1);
	point_counts = cvCreateMat(num_images, 1, CV_32SC1);

	// grab an image so we know the image size
	img = cvQueryFrame(capture);
	img_size = cvGetSize(img);

	rc = grab_calib_grid(capture, size, image_points, object_points, 
		point_counts, num_images);
	if(rc != num_images) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, num_images);
		return !CV_OK;
	}

	distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);
	intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	cvZero(intrinsic_matrix);
	cvZero(distortion_coeffs);

	cvCalibrateCamera2(object_points, image_points, point_counts, img_size, 
		intrinsic_matrix, distortion_coeffs);

	// Save the intrinsics and distortions
	cvSave("Intrinsics.xml", intrinsic_matrix );
	cvSave("Distortion.xml", distortion_coeffs );

	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);

	return CV_OK;
}

int get_camera_extrinsics(CvCapture *capture, int rows, int cols, 
						  CvPoint2D32f origin, float theta, double scale)
{
	int rc;
	int num_pts;
	CvSize grid_size;
	IplImage *img;
	CvPoint p;
	CvMat *wip, *wop, *wpp, *wpc;
	CvMat *intrinsic, *distortion;
	CvMat *Rvec, *Rmat, *Tvec;
	CvMat trans, obj_pts2;
	float trans_elem[] = {	cos(theta), -sin(theta), 0, origin.x, 
							sin(theta), cos(theta), 0, origin.y, 
							0, 0, 1, 0};


	img = NULL;
	num_pts = rows * cols;
	grid_size = cvSize(cols, rows);

	// allocate world image and object points and point counts
	wip = cvCreateMat(num_pts, 2, CV_32FC1);
	wop = cvCreateMat(num_pts, 3, CV_32FC1);
	wpp = cvCreateMat(num_pts, 2, CV_32FC1);
	wpc = cvCreateMat(ONE_EXTRINSIC_IMG, 1, CV_32SC1);
	if(!wip || !wop || !wpp || !wpc) return !CV_OK;

	// allocate extrensic parameters
	Rvec = cvCreateMat(3, 1, CV_32FC1);
	Rmat = cvCreateMat(3, 3, CV_32FC1);
	Tvec = cvCreateMat(3, 1, CV_32FC1);
	if(!Rvec || !Rmat || !Tvec) return !CV_OK;

	// load camera intrinsic model
	intrinsic = (CvMat*) cvLoad( "Intrinsics.xml" );
	distortion = (CvMat*) cvLoad( "Distortion.xml" );
	if(!intrinsic || !distortion) return !CV_OK;

	// get image and world points
	rc = grab_calib_grid(capture, grid_size, wip, wop, wpc, 
		ONE_EXTRINSIC_IMG, PROMPT, &img);
	if(rc != ONE_EXTRINSIC_IMG) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, 
			ONE_EXTRINSIC_IMG);
		return !CV_OK;
	}
	else if(img == NULL) {
		return !CV_OK;
	}

	// convert default coordinate system to new one using transformation matrix
	// TODO can obj_pts2 be replaced with wop?
	trans = cvMat(3, 4, CV_32FC1, trans_elem);
	cvReshape(wop, &obj_pts2, 3);
	cvTransform(&obj_pts2, &obj_pts2, &trans);
	cvScale(&obj_pts2, &obj_pts2, scale);

	// get extrinsic parameters
	cvFindExtrinsicCameraParams2(wop, wip, intrinsic, distortion, Rvec, Tvec);
	cvRodrigues2(Rvec, Rmat);
	cvProjectPoints2(wop, Rvec, Tvec, intrinsic, distortion, wpp);

	// draw image and world frame
	p = cvPoint(cvRound(cvmGet(intrinsic, 0, 2)), 
		cvRound(cvmGet(intrinsic, 1, 2)));
	draw_wrld2pxl(img, rows, cols, p, wop, wpp, wip);
	cvNamedWindow("world frame projected onto image frame", 0);
	cvResizeWindow("world frame projected onto image frame", 
		img->width, img->height);
	cvShowImage("world frame projected onto image frame", img);
	cvWaitKey(0);

	cvReleaseMat(&wop);
	cvReleaseMat(&wpp);
	cvReleaseMat(&wip);
	cvReleaseMat(&wpc);
	cvReleaseMat(&Rvec);
	cvReleaseMat(&Rmat);
	cvReleaseMat(&Tvec);

	return CV_OK;
}
