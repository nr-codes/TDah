// image open code taken from (also has useful indexing code)
// http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html#SECTION00025000000000000000
// last accessed 06/24/10

// grid orientation
//http://opencv-users.1802565.n2.nabble.com/FindChessboardCorners-ordering-td2122706.html
// last accessed 06/24/10

// calibration code
// http://dasl.mem.drexel.edu/~noahKuntz/openCVTut10.html
// last accessed 06/25/10

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
#define NUM_IMGS 1
#define PROMPT 1
#define NUM_POINTS (SQ_WIDTH*SQ_HEIGHT)

int main()
{
	int rc;
	CvSize grid_size;
	IplImage *imgs[NUM_IMGS];
	
	// world features and coordinates
	CvMat *world_Rvec = cvCreateMat(3, 1, CV_32FC1);
	CvMat *world_R = cvCreateMat(3, 3, CV_32FC1);
	CvMat *world_T = cvCreateMat(3, 1, CV_32FC1);

	CvMat *world_image_points = cvCreateMat(NUM_IMGS*NUM_POINTS, 2, CV_32FC1);
	CvMat *world_object_points = cvCreateMat(NUM_IMGS*NUM_POINTS, 3, CV_32FC1);
	CvMat *world_point_counts = cvCreateMat(NUM_IMGS, 1, CV_32SC1);
	grid_size = cvSize(SQ_WIDTH, SQ_HEIGHT);
	
	// get image and world points
	rc = grab4calib(SOURCE, world_object_points, world_image_points, 
		world_point_counts, &grid_size, NUM_IMGS, PROMPT, imgs);
	if(rc != NUM_IMGS) {
		// smart thing would be to resize everything, but...
		printf("didn't get all images, got %d out of %d\n", rc, NUM_IMGS);
		return -rc;
	}

	//cvSaveImage("extWebcam.tif", imgs[0]);
	/*
	// for use with matlab calib toolbox
	char text2[100];
	for(int i = 0; i < NUM_IMGS; i++) {
		sprintf_s(text2, 100, "Webcam%d.tif", i);
		cvSaveImage(text2, imgs[i]);
	}


	return 0;
	*/

	// camera model
	CvMat *intrinsic = (CvMat*)cvLoad( "../Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "../Distortion.xml" );
#if 0
	// matlab numbers
	CV_MAT_ELEM(*distortion, float, 0, 0) = 0.636135751509665f;
	CV_MAT_ELEM(*distortion, float, 1, 0) = -1.259910576683441f;
	CV_MAT_ELEM(*distortion, float, 2, 0) = -0.027239389565965f;
	CV_MAT_ELEM(*distortion, float, 3, 0) = 0.051650499081274f;

	CV_MAT_ELEM(*intrinsic, float, 0, 0) = 579.9827041049632f;
	CV_MAT_ELEM(*intrinsic, float, 1, 1) = 579.8742233379385f;
	CV_MAT_ELEM(*intrinsic, float, 0, 2) = 207.2964225086150f;
	CV_MAT_ELEM(*intrinsic, float, 1, 2) = 149.6309362650663f;
#endif
	PrintMat(intrinsic);
	PrintMat(distortion);

	// get extrinsic parameters
	cvFindExtrinsicCameraParams2(world_object_points, world_image_points, 
		intrinsic, distortion, world_Rvec, world_T);


	CvMat *world_proj_points = cvCreateMat(NUM_POINTS, 2, CV_32FC1);
	cvProjectPoints2(world_object_points, world_Rvec, world_T, 
		intrinsic, distortion, world_proj_points);
/*
	printf("(%f, %f, %f) -> (%f, %f)\n", CV_MAT_ELEM(*world_object_points, float, 0, 0),
		CV_MAT_ELEM(*world_object_points, float, 0, 1), 
		CV_MAT_ELEM(*world_object_points, float, 0, 2),
		CV_MAT_ELEM(*world_proj_points, float, 0, 0),
		CV_MAT_ELEM(*world_proj_points, float, 0, 1));
*/

	char text[100];
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN|CV_FONT_ITALIC,1,1,0,1);

	CvPoint2D32f corners[NUM_POINTS];
	for(int i = 0; i < NUM_POINTS; i++) {
		corners[i].x = CV_MAT_ELEM(*world_image_points, float, i, 0);
		corners[i].y = CV_MAT_ELEM(*world_image_points, float, i, 1);
	}

	cvDrawChessboardCorners(imgs[0], grid_size, corners, NUM_POINTS, rc);


	for(int i = 0; i < NUM_POINTS; i++) {
		int px = cvRound(CV_MAT_ELEM(*world_proj_points, float, i, 0));
		int py = cvRound(CV_MAT_ELEM(*world_proj_points, float, i, 1));
		cvDrawRect(imgs[0], cvPoint(px - 10, py - 10), 
			cvPoint(px + 10, py + 10), CV_RGB(128, 128, 128));
		cvDrawCircle(imgs[0], cvPoint(px, py), 2, CV_RGB(0,255, 255), CV_FILLED);

		int ox = cvRound(CV_MAT_ELEM(*world_object_points, float, i, 0));
		int oy = cvRound(CV_MAT_ELEM(*world_object_points, float, i, 1));
		memset(text, 0, 100);
		sprintf_s(text, 100, "(%d,%d)", ox, oy);
		cvPutText(imgs[0], text, cvPoint(px, py), &font, CV_RGB(0,255,0));
	}

	cvNamedWindow("reprojected");
	cvShowImage("reprojected", imgs[0]);

	CvMat *normalized = cvCreateMat(1, 1, CV_32FC2);
	CvMat *distorted = cvCreateMat(1, 1, CV_32FC2);
	CvMat *world = cvCreateMat(3, 1, CV_32FC1);
	CvMat *zero_vec = cvCreateMat(3, 1, CV_32FC1);
	cvZero(zero_vec);
	
	float fx = CV_MAT_ELEM(*intrinsic, float, 0, 0);
	float cx = CV_MAT_ELEM(*intrinsic, float, 0, 2);
	float fy = CV_MAT_ELEM(*intrinsic, float, 1, 1);
	float cy = CV_MAT_ELEM(*intrinsic, float, 1, 2);

	//PrintMat(intrinsic);
	float Z = 13.679170513009492f;
	cvRodrigues2(world_Rvec, world_R);
	CvMat *world_Rt = cvCloneMat(world_R);
	cvTranspose(world_R, world_Rt);

	cvDrawCircle(imgs[0], cvPoint(imgs[0]->width/2, imgs[0]->height/2), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvDrawCircle(imgs[0], cvPoint(cvRound(cx), cvRound(cy)), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvDrawCircle(imgs[0], cvPoint(cvRound(CV_MAT_ELEM(*world_image_points, float, 0, 0)), 
		cvRound(CV_MAT_ELEM(*world_image_points, float, 0, 1))), 4, 
		CV_RGB(255,0, 255), CV_FILLED);
	cvShowImage("reprojected", imgs[0]);

	//printf("pp: %f %f center: %d %d\n", cx, cy, imgs[0]->width/2, imgs[0]->height/2);

	CvMat *pp = cvCreateMat(1,1, CV_32FC3);
	CvMat *wp = cvCreateMat(1,1, CV_32FC3);

#if 0
	/*
  0.022591330594568   0.999184240472044   0.033473651941049
   0.964889263668658  -0.013027491198032  -0.262333744169599
  -0.261683665212261   0.038224835714346  -0.964396454418871
  */

	CV_MAT_ELEM(*world_R, float, 0, 0) = 0.022591330594568f;
	CV_MAT_ELEM(*world_R, float, 0, 1) = 0.999184240472044f;
	CV_MAT_ELEM(*world_R, float, 0, 2) = 0.033473651941049f;
	CV_MAT_ELEM(*world_R, float, 1, 0) = 0.964889263668658f;
	CV_MAT_ELEM(*world_R, float, 1, 1) = -0.013027491198032f;
	CV_MAT_ELEM(*world_R, float, 1, 2) = -0.262333744169599f;
	CV_MAT_ELEM(*world_R, float, 2, 0) = -0.261683665212261f;
	CV_MAT_ELEM(*world_R, float, 2, 1) = 0.038224835714346f;
	CV_MAT_ELEM(*world_R, float, 2, 2) = -0.964396454418871f;
	cvRodrigues2(world_R, world_Rvec);

	/*
	-3.644665939992220
  -0.425116091855695
  15.174287266210367
  */
	CV_MAT_ELEM(*world_T, float, 0, 0) = -3.644665939992220f;
	CV_MAT_ELEM(*world_T, float, 1, 0) = -0.425116091855695f;
	CV_MAT_ELEM(*world_T, float, 2, 0) = 15.174287266210367f;
#endif

	PrintMat(world_R);
	PrintMat(world_T);

	for(int i = 0; i < NUM_POINTS; i++) {
		float u = CV_MAT_ELEM(*world_image_points, float, i, 0);
		float v = CV_MAT_ELEM(*world_image_points, float, i, 1);

		
		printf("%d (%f, %f, %f) -> (%f, %f)\n", i,
			CV_MAT_ELEM(*world_object_points, float, i, 0),
			CV_MAT_ELEM(*world_object_points, float, i, 1), 
			CV_MAT_ELEM(*world_object_points, float, i, 2),
			u, v);

		float xp = u;//(u - cx)/fx;
		float yp = v;//(v - cy)/fy;

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
		cvSub(world, world_T, world);
		cvGEMM(world_R, world, 1, NULL, 0, world, CV_GEMM_A_T);
		wp->data.fl[0] = CV_MAT_ELEM(*world, float, 0, 0);
		wp->data.fl[1] = CV_MAT_ELEM(*world, float, 1, 0);
		wp->data.fl[2] = CV_MAT_ELEM(*world, float, 2, 0);

		cvProjectPoints2(wp, world_Rvec, world_T, 
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

	cvReleaseMat(&world_object_points);
	cvReleaseMat(&world_image_points);
	cvReleaseMat(&world_point_counts);
	cvReleaseMat(&world_Rvec);
	cvReleaseMat(&world_R);
	cvReleaseMat(&world_T);
	cvReleaseMat(&normalized);
	cvReleaseMat(&distorted);
	cvReleaseMat(&world);

	return 0;
}
