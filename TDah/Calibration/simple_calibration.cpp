#include <cv.h>
#include <iostream>
#include "Calibration.h"

#define GRID_ROWS 5
#define GRID_COLS 7

#define INTRINSIC 0
#define EXTRINSIC 1

#define EXTRINSIC_FILE "ExtrinsicCamera.yaml"
#define INTRINSIC_FILE "IntrinsicCamera.yaml"
#define WORLD_FILE "worldpoints.txt"

using namespace cv;

extern CvCapture* cvCreateCameraCapture_CMU (int index);

int intrinsic(Calibration& calib, CvCapture* cap, vector<Point3f>& world)
{
	Mat img;

	/** Intrinsic Calibration **/
	// grab images for calibration
	img = Mat(cvQueryFrame(cap));
	if(img.empty()) {
		return -3;
	}
	calib.getClickViews(cap, world);

	size_t n = calib.views.pixel.size();
	if( !n ) {
		return -4;
	}
	calib.views.world.assign(n, world);
	calib.getIntrinsics(img.size());

	// we are responsible for clearing all vectors
	// prior to doing another calibration, otherwise
	// previous elements stay in the vectors
	calib.clearVectors();

	// write to file
	CvFileStorage* fs = cvOpenFileStorage(INTRINSIC_FILE, NULL, CV_STORAGE_WRITE);
	if(fs == NULL) {
		return -6;
	}
	
	// old tracking code expects 32-bit floats
	calib.intrinsic_params.A.convertTo(calib.intrinsic_params.A, CV_32F);
	calib.intrinsic_params.k.convertTo(calib.intrinsic_params.k, CV_32F);
	
	CvMat A = calib.intrinsic_params.A;
	CvMat k = calib.intrinsic_params.k;

	write_intrinsic_params(fs, &A, &k);
	cvReleaseFileStorage(&fs);

	return 0;
}

int extrinsic(Calibration& calib, CvCapture* cap, vector<Point3f>& world)
{
	// read in intrinsic parameters
	CvMat* A;
	CvMat* k;

	// read intrinsic file
	CvFileStorage* fs = cvOpenFileStorage(INTRINSIC_FILE, NULL, CV_STORAGE_READ);
	if(fs == NULL) {
		return -6;
	}

	read_intrinsic_params(fs, &A, &k);
	if(A == NULL || k == NULL) {
		return -5;
	}

	Mat(A).convertTo(calib.intrinsic_params.A, CV_64F);
	Mat(k).convertTo(calib.intrinsic_params.k, CV_64F);
	cvReleaseFileStorage(&fs);

	// grab an image for our world frame
	Mat Tr = Mat::eye(3, 4, CV_64FC1);
	calib.getClickViews(cap, world);
	size_t n = calib.views.pixel.size();
	if( !n ) {
		return -6;
	}
	calib.views.world.assign(n, world);
	calib.getExtrinsics(Tr);

	// we are responsible for clearing all vectors
	// prior to doing another calibration, otherwise
	// previous elements stay in the vectors
	calib.clearVectors();

	// write to file
	fs = cvOpenFileStorage(EXTRINSIC_FILE, NULL, CV_STORAGE_WRITE);
	if(fs == NULL) {
		return -7;
	}
	
	// old tracking code expects 32-bit floats
	calib.extrinsic_params.R.convertTo(calib.extrinsic_params.R, CV_32F);
	calib.extrinsic_params.t.convertTo(calib.extrinsic_params.t, CV_32F);

	CvMat R = calib.extrinsic_params.R;
	CvMat t = calib.extrinsic_params.t;

	write_extrinsic_params(fs, &R, &t);
	cvReleaseFileStorage(&fs);

	return 0;
}

int main()
{
	int rc;
	Mat img;
	vector<Point3f> world;
	CvCapture* cap = cvCreateCameraCapture_CMU(0);

	Size grid = Size(GRID_COLS, GRID_ROWS);
	Calibration calib(grid); // the chessboard is a 3x4 grid

	if(!calib.getWorldPoints(WORLD_FILE, world)) {
		return -1;
	}
	else if(world.size() != grid.area()) {
		return -2;
	}

	rc = 0;
	if(INTRINSIC) {
		rc = intrinsic(calib, cap, world);
		if(rc < 0) {
			return rc;
		}
	}
	
	if(EXTRINSIC) {
		rc = extrinsic(calib, cap, world);
	}

	cvReleaseCapture(&cap);

	return rc;
}