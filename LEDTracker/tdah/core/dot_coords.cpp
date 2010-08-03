#include <errno.h>
#include "t_dah.h"

#define WIN_SIZE cvSize(5, 5)
#define ZERO_ZNE cvSize(-1, -1)
#define ERR_TOL cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1)
#define NUM_IMGS_EXTRINSIC 1
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
					CvMat *ip, CvMat *op, CvMat *pc, int num_imgs, int prompt,
					IplImage *store_imgs[])
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
	if(img == NULL) return !CV_OK;
	gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	if(gray == NULL) return !CV_OK;
	corners = (CvPoint2D32f *) cvAlloc(sizeof(CvPoint2D32f) * num_points);
	if(corners == NULL) return -ENOMEM;

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

				cvmSet(op, row0 + i, 0, i / cols);
				cvmSet(op, row0 + i, 1, i % cols);
				cvmSet(op, row0 + i, 2, 0);
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

int get_camera_intrinsics(CvCapture *capture, int rows, int cols, int num_images, 
						  CvPoint2D32f origin, float theta)
{
	int rc, num_points;
	IplImage *img = NULL;
	CvSize size, img_size;
	CvMat trans, obj_pts2;
	CvMat *image_points, *object_points, *point_counts;
	CvMat *intrinsic_matrix, *distortion_coeffs;
	float trans_elem[] = {	cos(theta), -sin(theta), 0, origin.x, 
							sin(theta), cos(theta), 0, origin.y, 
							0, 0, 1, 0};

	// allocate calibration data
	num_points = rows * cols;
	size = cvSize(cols, rows);

	image_points = cvCreateMat(num_images*num_points, 2, CV_32FC1);
	object_points = cvCreateMat(num_images*num_points, 3, CV_32FC1);
	point_counts = cvCreateMat(num_images, 1, CV_32SC1);

	cvReshape(object_points, &obj_pts2, 3);
	trans = cvMat(3, 4, CV_32FC1, trans_elem);

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
	
	cvTransform(&obj_pts2, &obj_pts2, &trans);
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
						  CvPoint2D32f origin, float theta)
{
	int rc;
	int num_pts;
	CvSize grid_size;
	IplImage *img;

	num_pts = rows * cols;
	grid_size = cvSize(cols, rows);
	
	// world features and coordinates
	CvMat *Rvec = cvCreateMat(3, 1, CV_32FC1);
	CvMat *Rmat = cvCreateMat(3, 3, CV_32FC1);
	CvMat *Tvec = cvCreateMat(3, 1, CV_32FC1);

	CvMat *wip = cvCreateMat(NUM_IMGS_EXTRINSIC*num_pts, 2, CV_32FC1);
	CvMat *wop = cvCreateMat(NUM_IMGS_EXTRINSIC*num_pts, 3, CV_32FC1);
	CvMat *wpc = cvCreateMat(NUM_IMGS_EXTRINSIC, 1, CV_32SC1);
	
	
	// get image and world points
	rc = grab_calib_grid(capture, grid_size, wip, wop, wpc, 
		NUM_IMGS_EXTRINSIC, PROMPT, &img);
	if(rc != NUM_IMGS_EXTRINSIC) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, NUM_IMGS_EXTRINSIC);
		return !CV_OK;
	}

	// camera model
	CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );

	// get extrinsic parameters
	cvFindExtrinsicCameraParams2(wop, wip, intrinsic, distortion, Rvec, Tvec);

	CvMat *wpp = cvCreateMat(num_pts, 2, CV_32FC1);
	cvProjectPoints2(wop, Rvec, Tvec, intrinsic, distortion, wpp);

	char text[100];
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,1,0,1);

	CvPoint2D32f *corners = (CvPoint2D32f *) cvAlloc(num_pts * sizeof(CvPoint2D32f));
	for(int i = 0; i < num_pts; i++) {
		corners[i].x = (float) cvmGet(wip, i, 0);
		corners[i].y = (float) cvmGet(wip, i, 1);
	}

	cvDrawChessboardCorners(img, grid_size, corners, num_pts, rc);

	for(int i = 0; i < num_pts; i++) {
		int px = cvRound(cvmGet(wpp, i, 0));
		int py = cvRound(cvmGet(wpp, i, 1));
		cvDrawRect(img, cvPoint(px - 10, py - 10), 
			cvPoint(px + 10, py + 10), CV_RGB(128, 128, 128));
		cvDrawCircle(img, cvPoint(px, py), 2, CV_RGB(0,255, 255), CV_FILLED);

		int ox = cvRound(cvmGet(wop, i, 0));
		int oy = cvRound(cvmGet(wop, i, 1));
		memset(text, 0, 100);
		sprintf_s(text, 100, "(%d,%d)", ox, oy);
		cvPutText(img, text, cvPoint(px, py), &font, CV_RGB(0,255,0));
	}

	cvNamedWindow("reprojected");
	cvShowImage("reprojected", img);

	CvMat *normalized = cvCreateMat(1, 1, CV_32FC2);
	CvMat *distorted = cvCreateMat(1, 1, CV_32FC2);
	CvMat *world = cvCreateMat(3, 1, CV_32FC1);
	CvMat *zero_vec = cvCreateMat(3, 1, CV_32FC1);
	cvZero(zero_vec);
	
	float fx = CV_MAT_ELEM(*intrinsic, float, 0, 0);
	float cx = CV_MAT_ELEM(*intrinsic, float, 0, 2);
	float fy = CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float cy = CV_MAT_ELEM(*intrinsic, float, 1, 2);


	float Z = 13.679170513009492f;
	cvRodrigues2(Rvec, Rmat);
	CvMat *world_Rt = cvCloneMat(Rmat);
	cvTranspose(Rmat, world_Rt);

	cvDrawCircle(img, cvPoint(img->width/2, img->height/2), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvDrawCircle(img, cvPoint(cvRound(cx), cvRound(cy)), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvDrawCircle(img, cvPoint(cvRound(CV_MAT_ELEM(*wip, float, 0, 0)), 
		cvRound(CV_MAT_ELEM(*wip, float, 0, 1))), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvShowImage("reprojected", img);

	CvMat *pp = cvCreateMat(1,1, CV_32FC3);
	CvMat *wp = cvCreateMat(1,1, CV_32FC3);

	for(int i = 0; i < num_pts; i++) {
		float u = CV_MAT_ELEM(*wip, float, i, 0);
		float v = CV_MAT_ELEM(*wip, float, i, 1);

		
		printf("%d (%f, %f, %f) -> (%f, %f)\n", i,
			CV_MAT_ELEM(*wop, float, i, 0),
			CV_MAT_ELEM(*wop, float, i, 1), 
			CV_MAT_ELEM(*wop, float, i, 2),
			u, v);

		float xp = u;
		float yp = v;

		// distorted to undistorted normalized coordinates
		distorted->data.fl[0] = xp;
		distorted->data.fl[1] = yp;
		cvUndistortPoints(distorted, normalized, intrinsic, distortion);

		CV_MAT_ELEM(*world, float, 0, 0) = normalized->data.fl[0];
		CV_MAT_ELEM(*world, float, 1, 0) = normalized->data.fl[1];
		CV_MAT_ELEM(*world, float, 2, 0) = 1;

		printf("(%f, %f) -> (%f, %f)\n", 
			distorted->data.fl[0], distorted->data.fl[1], 
			normalized->data.fl[0], normalized->data.fl[1]);

		wp->data.fl[0] = CV_MAT_ELEM(*world, float, 0, 0);
		wp->data.fl[1] = CV_MAT_ELEM(*world, float, 1, 0);
		wp->data.fl[2] = CV_MAT_ELEM(*world, float, 2, 0);
		cvProjectPoints2(wp, zero_vec, zero_vec, intrinsic, distortion, pp);

		printf("(%f, %f) <- (%f, %f)\n", 
			pp->data.fl[0], 
			pp->data.fl[1], 
			wp->data.fl[0], 
			wp->data.fl[1]);

		
		// world to image plane
		CV_MAT_ELEM(*world, float, 0, 0) = Z*normalized->data.fl[0];
		CV_MAT_ELEM(*world, float, 1, 0) = Z*normalized->data.fl[1];
		CV_MAT_ELEM(*world, float, 2, 0) = Z*1;
		cvSub(world, Tvec, world);
		cvGEMM(Rmat, world, 1, NULL, 0, world, CV_GEMM_A_T);
		wp->data.fl[0] = CV_MAT_ELEM(*world, float, 0, 0);
		wp->data.fl[1] = CV_MAT_ELEM(*world, float, 1, 0);
		wp->data.fl[2] = CV_MAT_ELEM(*world, float, 2, 0);

		cvProjectPoints2(wp, Rvec, Tvec, 
			intrinsic, distortion, pp);
		printf("(%f, %f) <- %f %f %f\n",
			fx*pp->data.fl[0] + cx, 
			fy*pp->data.fl[1] + cy, 
			wp->data.fl[0], 
			wp->data.fl[1], 
			wp->data.fl[2]);

		printf("\n");
	}

	cvWaitKey(0);

	cvFree(&corners);
	cvReleaseMat(&wop);
	cvReleaseMat(&wip);
	cvReleaseMat(&wpc);
	cvReleaseMat(&Rvec);
	cvReleaseMat(&Rmat);
	cvReleaseMat(&Tvec);
	cvReleaseMat(&normalized);
	cvReleaseMat(&distorted);
	cvReleaseMat(&world);

	return 0;
}
