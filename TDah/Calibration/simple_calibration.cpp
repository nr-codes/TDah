#include <cv.h>
#include <iostream>
#include "Calibration.h"

using cv::Mat;
using cv::Scalar;
using cv::VideoCapture;

bool simple_intrinsic(Calibration& calib, VideoCapture* cap)
{
	int n;
	Mat img;

	// make sure all vectors are emptied otherwise
	// data from previous runs might still exist
	calib.clearVectors();

	// make sure camera is opened
	*cap >> img;
	if(img.empty()) {
		return false;
	}

	// setup the world frame using the classic x-y coordinate system
	calib.xyWorldPoints();

	// get chessboard views
	n = calib.getClickViews(cap, "intrinsic");
	if( n != calib.views.n ) {
		return false;
	}

	// get and save the intrinsic parameters
	calib.getIntrinsics(img.size());
	return calib.writeIntrinsics();
}

bool simple_extrinsic(Calibration& calib, VideoCapture* cap, Mat& Tr)
{
	int n;

	// make sure all vectors are emptied otherwise
	// data from previous runs might still exist
	calib.clearVectors();

	if(Tr.empty()) {
		Tr = Mat::eye(3, 4, CV_64FC1);
	}

	// read in intrinsic parameters
	if(!calib.readIntrinsics())
		return false;

	// setup the world frame using the classic x-y coordinate system
	calib.xyWorldPoints();

	// grab one image for our world frame, but prompt
	// the user to reject (press the 'i' key) or accept
	// (press any other key) the image.  Also, save the
	// image for later processing
	calib.views.n = 1;
	calib.views.prompt = true;
	calib.views.save_views = true;
	n = calib.getClickViews(cap, "extrinsic");
	if( n != calib.views.n ) {
		return false;
	}

	// get the extrinsic parameters and write to file
	calib.getExtrinsics(Tr);
	if(!calib.writeExtrinsics())
		return false;

	// the reproject image windows remains open after
	// the function returns, so let the user look at the
	// image until they press a key
	Scalar red = Scalar(0, 0, 255);
	Scalar green = Scalar(0, 255, 0);
	calib.reproject(red, green);
	cv::waitKey();

	return true;
}

#if 0 // the following is example code on how to use the two functions
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
	// relative paths should be relative to the ./build/your_build_dir/
	calib.intrinsic_params.file = "../../Intrinsics.yaml";
	calib.extrinsic_params.file = "../../Extrinsics.yaml";
	if(do_intrinsic)
		simple_intrinsic(calib, &cap);

	if(do_extrinsic) {
		// setup the transformation matrix, which is
		// useful if the new world frame should be 
		// rotated or translated from the original
		// world points (currently nothing is applied)
		// See the OpenCV "Basic Structures" documentation
		// for other ways to initialize a Mat object.
		Mat Tr = Mat::eye(3, 4, CV_64FC1);
		simple_extrinsic(calib, &cap, Tr);
	}

	return 0;
}
#endif