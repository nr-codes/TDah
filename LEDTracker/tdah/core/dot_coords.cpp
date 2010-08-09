#include <errno.h>
#include "t_dah.h"

#define WIN_SIZE cvSize(5, 5)
#define ZERO_ZNE cvSize(-1, -1)
#define ERR_TOL cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1)
#define ONE_EXTRINSIC_IMG 1
#define PROMPT 1

CvPoint world2pixel(CvPoint2D32f w, CvMat *A, CvMat *k, CvMat *R, CvMat *T)
{
	// all code based on OpenCV cvProjectPoints2 function
	CvPoint p_i;
	double _A[9], _k[5] = {0, 0, 0, 0, 0}, _R[9], _T[3];
    CvMat matA, matk, matR, matT;
	double X, Y, Z, x, y, z;
    double r2, r4, r6, a1, a2, a3, cdist;
    double xd, yd, fx, fy, cx , cy;

	// convert matrices from float to double
	matA = cvMat(3, 3, CV_64F, _A);
	cvConvert(A, &matA);

	matk = cvMat(4, 1, CV_64F, _k);
	cvConvert(k, &matk);

	matR = cvMat(3, 3, CV_64F, _R);
	cvConvert(R, &matR);

	matT = cvMat(3, 1, CV_64F, _T);
	cvConvert(T, &matT);

	// world frame
	X = (double) w.x;
	Y = (double) w.y;
	Z = 0;

	// camera frame
    x = _R[0]*X + _R[1]*Y + _R[2]*Z + _T[0];
    y = _R[3]*X + _R[4]*Y + _R[5]*Z + _T[1];
    z = _R[6]*X + _R[7]*Y + _R[8]*Z + _T[2];

	// image frame
    z = z ? 1./z : 1;
    x *= z; 
	y *= z;

	fx = _A[0];
	fy = _A[4];
    cx = _A[2];
	cy = _A[5];

    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + _k[0]*r2 + _k[1]*r4 + _k[4]*r6;
    xd = x*cdist + _k[2]*a1 + _k[3]*a2;
    yd = y*cdist + _k[2]*a3 + _k[3]*a1;

    p_i.x = cvRound(xd*fx + cx);
    p_i.y = cvRound(yd*fy + cy);

	return p_i;
}

CvPoint2D32f pixel2world(CvPoint p, CvMat *A, CvMat *k, CvMat *R, CvMat *T)
{
	CvPoint2D32f p_w;
	CvMat world_coord, img_coord;
	float img_crd[2] = {(float) p.x, (float) p.y};
	float w_crd[3];
	float z;

	// convert from distorted pixels to normalized camera frame
	img_coord = cvMat(1, 1, CV_32FC2, img_crd);
	cvUndistortPoints(&img_coord, &img_coord, A, k);

	// convert from camera frame to world frame
	z = T->data.fl[2];
	world_coord = cvMat(3, 1, CV_32FC1, w_crd);
	cvmSet(&world_coord, 0, 0, z*img_coord.data.fl[0] - T->data.fl[0]);
	cvmSet(&world_coord, 1, 0, z*img_coord.data.fl[1] - T->data.fl[1]);
	cvmSet(&world_coord, 2, 0, 0);

	//cvSub(&world_coord, T, &world_coord);
	cvGEMM(R, &world_coord, 1, NULL, 0, &world_coord, CV_GEMM_A_T);

	p_w.x = world_coord.data.fl[0];
	p_w.y = world_coord.data.fl[1];

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
					// by default use normal cartesian coord system
					cvmSet(op, row0 + i, 0, i % cols); // x goes left to right
					cvmSet(op, row0 + i, 1, -i / cols); // y goes top to bottom
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

// TODO consider using initial guess
int get_camera_intrinsics(CvCapture *capture, char *file, 
						  int rows, int cols, int num_images)
{
	int rc, num_points;
	IplImage *img = NULL;
	CvFileStorage *fs = NULL;
	CvSize size, img_size;
	CvMat *image_points, *object_points, *point_counts;
	CvMat *intrinsic_matrix, *distortion_coeffs;
	
	// allocate calibration data
	num_points = rows * cols;
	size = cvSize(cols, rows);

	// assume error
	rc = !CV_OK;

__CV_BEGIN__;

	image_points = cvCreateMat(num_images*num_points, 2, CV_32FC1);
	object_points = cvCreateMat(num_images*num_points, 3, CV_32FC1);
	point_counts = cvCreateMat(num_images, 1, CV_32SC1);
	if(!image_points || !object_points || !point_counts) __CV_EXIT__;

	distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);
	intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	if(!distortion_coeffs || !intrinsic_matrix) __CV_EXIT__;

	// grab an image so we know the image size
	img = cvQueryFrame(capture);
	if(!img) __CV_EXIT__;
	img_size = cvGetSize(img);

	if(grab_calib_grid(capture, size, image_points, object_points, 
		point_counts, num_images) != num_images) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, num_images);
		__CV_EXIT__;
	}

	cvZero(intrinsic_matrix);
	cvZero(distortion_coeffs);
	cvCalibrateCamera2(object_points, image_points, point_counts, img_size, 
		intrinsic_matrix, distortion_coeffs);

	// Save the intrinsics and distortions
	if(file == NULL) {
		//cvSave("Intrinsics.xml", intrinsic_matrix );
		//cvSave("Distortion.xml", distortion_coeffs );
	}
	else {
		fs = cvOpenFileStorage(file, NULL, CV_STORAGE_WRITE);
		if(!fs) __CV_EXIT__;
		write_intrinsic_params(fs, intrinsic_matrix, distortion_coeffs);
	}

	rc = CV_OK;

__CV_END__;
	cvReleaseFileStorage(&fs);
	cvReleaseMat(&image_points);
	cvReleaseMat(&object_points);
	cvReleaseMat(&point_counts);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);

	return rc;
}

// TODO consider using initial guess
int get_camera_extrinsics(CvCapture *capture, char *save_file, char *intrinsic_file,
						  int rows, int cols, double scale,
						  CvPoint2D32f newOrigin, float theta)
{
	int rc;
	int num_pts;
	CvSize grid_size;
	CvFileStorage *fs = NULL;
	IplImage *img;
	CvPoint p;
	CvMat *pp, *wp;
	CvMat *wip, *wop, *wpp, *wpc;
	CvMat *intrinsic, *distortion;
	CvMat *Rvec, *Rmat, *Tvec;
	CvMat rot, shift;
	float rot_elem[] = {cos(theta), sin(theta), 0,
						-sin(theta), cos(theta), 0, 
						0, 0, 1};
	float shift_elem[] = {	1, 0, 0, -newOrigin.x, 
							0, 1, 0, -newOrigin.y, 
							0, 0, 1, 0};


	img = NULL;
	num_pts = rows * cols;
	grid_size = cvSize(cols, rows);

	// assume error
	rc = !CV_OK;

__CV_BEGIN__;

	fs = cvOpenFileStorage(intrinsic_file, NULL, CV_STORAGE_READ);
	if(!fs) __CV_EXIT__;

	// allocate world image and object points and point counts
	wip = cvCreateMat(num_pts, 2, CV_32FC1);
	wop = cvCreateMat(num_pts, 3, CV_32FC1);
	wpp = cvCreateMat(num_pts, 2, CV_32FC1);
	wpc = cvCreateMat(ONE_EXTRINSIC_IMG, 1, CV_32SC1);
	if(!wip || !wop || !wpp || !wpc) __CV_EXIT__;

	// allocate extrensic parameters
	Rvec = cvCreateMat(3, 1, CV_32FC1);
	Rmat = cvCreateMat(3, 3, CV_32FC1);
	Tvec = cvCreateMat(3, 1, CV_32FC1);
	if(!Rvec || !Rmat || !Tvec) __CV_EXIT__;

	// allocate origin of world and image points for drawing
	pp = cvCreateMat(1,1, CV_32FC2);
	wp = cvCreateMat(1,1, CV_32FC3);
	if(!pp || !wp) __CV_EXIT__;

	// load camera intrinsic model
	read_intrinsic_params(fs, &intrinsic, &distortion);
	if(!intrinsic || !distortion) __CV_EXIT__;
	cvReleaseFileStorage(&fs);

	// get image and world points
	if(grab_calib_grid(capture, grid_size, wip, wop, wpc, 
		ONE_EXTRINSIC_IMG, PROMPT, &img) != ONE_EXTRINSIC_IMG) __CV_EXIT__;

	// convert default coordinate system to new one
	shift = cvMat(3, 4, CV_32FC1, shift_elem);
	rot = cvMat(3, 3, CV_32FC1, rot_elem);
	cvReshape(wop, wop, 3);
	cvTransform(wop, wop, &shift);
	cvTransform(wop, wop, &rot);
	cvScale(wop, wop, scale);
	cvReshape(wop, wop, 1);

	// get extrinsic parameters
	cvFindExtrinsicCameraParams2(wop, wip, intrinsic, distortion, Rvec, Tvec);
	cvRodrigues2(Rvec, Rmat);

	// save parameters
	fs = cvOpenFileStorage(save_file, NULL, CV_STORAGE_WRITE);
	if(!fs) __CV_EXIT__;
	write_extrinsic_params(fs, Rmat, Tvec);

	// draw projected world frame's axis
	wp->data.fl[0] = 0;
	wp->data.fl[1] = 0;
	wp->data.fl[2] = 0;
	cvProjectPoints2(wp, Rvec, Tvec, intrinsic, distortion, pp);
	p.x = cvRound(pp->data.fl[0]);
	p.y = cvRound(pp->data.fl[1]);
	cvTranspose(Rmat, Rmat);
	//draw_axis(img, "xy", p, Rmat);

	// draw projected world frame points
	cvProjectPoints2(wop, Rvec, Tvec, intrinsic, distortion, wpp);
	//draw_wrld2pxl(img, rows, cols, wop, wpp, wip);
	draw_wrld2pxl(img, "xy", intrinsic, distortion, Rmat, Tvec);

	cvNamedWindow("world frame projected onto image frame", 0);
	cvResizeWindow("world frame projected onto image frame", 
		img->width, img->height);
	cvShowImage("world frame projected onto image frame", img);
	cvWaitKey(0);

	rc = CV_OK;

__CV_END__;
	cvReleaseFileStorage(&fs);
	cvReleaseMat(&wp);
	cvReleaseMat(&pp);
	cvReleaseMat(&wop);
	cvReleaseMat(&wpp);
	cvReleaseMat(&wip);
	cvReleaseMat(&wpc);
	cvReleaseMat(&Rvec);
	cvReleaseMat(&Rmat);
	cvReleaseMat(&Tvec);

	return rc;
}
