#include <iostream>
#include <cv.h>

#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlgs/TrackDot.h"
#include "Calibration.h"

#define NDOTS 3
#define ROIW 20
#define ROIH 20
#define NIMGS 50

using namespace cv;

static bool calibrate(VideoCapture& cap, Camera& cam);

int main()
{
	// all images are Mat objects in OpenCV's C++ documentation
	Mat img;

	// choose a video source and tracking algorithm
	VideoCapture webcam(0); // use the default OpenCV camera source, e.g. webcam
	TrackDot alg(ROIW, ROIH, CV_THRESH_BINARY_INV); // use a dot tracking alg

	// setup the tracking system
	Dots dots(NDOTS); // create n dots to track
	Camera cam(webcam); // initialize the camera with the video source
	Tracker tracker(alg); // initialize the tracker with the traking alg

	// ** calibrate the camera **
	// the calibrate function should be treated as script file (like in Matlab)
	// just change the parameters in the function to control its behavior
	if(!calibrate(webcam, cam)) {
		return -1;
	}

	// get initial positions of all NDOTS by user-clicks
	dots.makeAllDotsActive(); // only active dots are updated/modified
	if(NDOTS != tracker.click(cam, dots)) {
		// quit if not all dots have been clicked on
		return -2;
	}

	// track dots across NIMGS images and quit demo
	for(int i = 1; i <= NIMGS; ++i) {
		// grab the next image and add the dots to the active set
		// note that the parameters are handled internally by the camera 
		// object and are only estimates, since the underlying VideoCapture 
		// does not provide any information at all
		if(!cam.grab(dots)) {
			return -3;
		}

		// track and show the dots
		tracker.track(cam, dots);
		tracker.draw(cam, dots, img);
		imshow("Dots", img);
		waitKey(1);

		// print out location information of active dots
		std::cout << tracker.str(dots);
	}
	
	return 0;
}

bool calibrate(VideoCapture& cap, Camera& cam)
{
	int nimgs = 1; // number of images
	int rows = 4;
	int cols = 3;

	// what type of calibration should be done?
	bool do_intrinsic = true;
	bool do_extrinsic = true;

	// create the calibration structure and the camera
	Calibration calib(Size(rows, cols), nimgs);

	// what are the file names?
	// relative paths are relative to the ./build/vs2k8 directory
	calib.intrinsic_params.file = "../../Intrinsics.yaml";
	calib.extrinsic_params.file = "../../Extrinsics.yaml";

	// perform the necessary calibration
	bool rci = true; // keeps track of intrinsic outcome
	if(do_intrinsic) {
		rci = simple_intrinsic(calib, &cap);
		cvDestroyAllWindows();
	}

	bool rce = true; // keeps track of extrinsic outcome
	if(rci && do_extrinsic) {
		// setup the transformation matrix, which is
		// useful if the new world frame should be 
		// rotated or translated from the original
		// world points (currently nothing is applied)
		// See the OpenCV "Basic Structures" documentation
		// for other ways to initialize a Mat object.
		Mat Tr = Mat::eye(3, 4, CV_64FC1);
		rce = simple_extrinsic(calib, &cap, Tr);
		cvDestroyAllWindows();
	}

	// load calibration parameters from files
	if(calib.readIntrinsics() && calib.readExtrinsics()) {
		// setup world frame
		cam.setA(calib.intrinsic_params.A);
		cam.setK(calib.intrinsic_params.k);
		cam.setR(calib.extrinsic_params.R);
		cam.setT(calib.extrinsic_params.t);
		
		// return true if new calibration was done
		// and was successful or if the files exist
		// and no new calibration was attempted
		return rci && rce;
	}

	return false;
}
