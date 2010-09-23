#include <cv.h>
#include <iostream>
#include "Calibration.h"

using namespace cv;

static int intrinsic(Calibration& calib, VideoCapture* cap);
static int extrinsic(Calibration& calib, VideoCapture* cap);

int main()
{
	int nimgs = 5; // number of images
	int rows = 6;
	int cols = 5;

	// what type of calibration should be done?
	bool do_intrinsic = true;
	bool do_extrinsic = true;

	// create the calibration structure and the camera
	Calibration calib(Size(rows, cols), nimgs);
	VideoCapture cap(0);

	// what are the file names?
	calib.intrinsic_params.file = "../../Intrinsics.yaml";
	calib.extrinsic_params.file = "../../Extrinsics.yaml";
	if(do_intrinsic)
		intrinsic(calib, &cap);

	if(do_extrinsic)
		extrinsic(calib, &cap);

	return 0;
}

int intrinsic(Calibration& calib, VideoCapture* cap)
{
	Mat img;

	// make sure all vectors are emptied otherwise
	// data from previous runs might still exist
	calib.clearVectors();

	// make sure camera is opened
	*cap >> img;
	if(img.empty()) {
		return -3;
	}

	// get chessboard views
	calib.getChessboardViews(cap);
	if( calib.views.pixel.empty() ) {
		return -4;
	}

	// setup the world frame using the classic x-y coordinate system
	calib.xyWorldPoints();

	// get and save the intrinsic parameters
	calib.getIntrinsics(img.size());
	calib.writeIntrinsics();

	return 0;
}

int extrinsic(Calibration& calib, VideoCapture* cap)
{
	// make sure all vectors are emptied otherwise
	// data from previous runs might still exist
	calib.clearVectors();

	// read in intrinsic parameters
	calib.readIntrinsics();

	// setup the transformation matrix, which is
	// useful if the new world frame should be 
	// rotated or translated from the original
	// world points (currently nothing is applied)
	// See the OpenCV "Basic Structures" documentation
	// for other ways to initialize a Mat object.
	Mat Tr = Mat::eye(3, 4, CV_64FC1);

	// grab one image for our world frame, but prompt
	// the user to reject (press the 'i' key) or accept
	// (press any other key) the image.  Also, save the
	// image for later processing
	calib.views.n = 1;
	calib.views.prompt = true;
	calib.views.save_views = true;
	calib.getChessboardViews(cap);
	if( calib.views.pixel.empty() ) {
		return 1;
	}

	// get the extrinsic parameters and write to file
	calib.xyWorldPoints();
	calib.getExtrinsics(Tr);
	calib.writeExtrinsics();

	// the reproject image windows remains open after
	// the function returns, so let the user look at the
	// image until they press a key
	Scalar red = Scalar(0, 0, 255);
	Scalar green = Scalar(0, 255, 0);
	calib.reproject(red, green);
	cv::waitKey();

	return 0;
}