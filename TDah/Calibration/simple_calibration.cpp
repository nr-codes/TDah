#include <cv.h>
#include <iostream>
#include "Calibration.h"

using namespace cv;

#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"

int main()
{
	Mat img;
	vector<Point3f> world;
	VideoCapture cap(0);
	Size grid = Size(4, 3);
	Calibration calib(grid); // the chessboard is a 3x4 grid

	if(!calib.getWorldPoints("worldpoints.txt", world)) {
		return -1;
	}
	else if(world.size() != grid.area()) {
		return -2;
	}

	/** Intrinsic Calibration **/
	// grab images for calibration
	cap >> img;
	calib.getClickViews(cap);

	size_t n = calib.views.pixel.size();
	calib.views.world.assign(n, world);
	calib.getIntrinsics(img.size());

	// intrinsic parameters are now ready to access
	calib.intrinsic_params.A;
	calib.intrinsic_params.k;

	// we are responsible for clearing all vectors
	// prior to doing another calibration, otherwise
	// previous elements stay in the vectors
	calib.clearVectors();

	/** Extrinsic Calibration **/
	// grab an image for our world frame
	Mat Tr = Mat::eye(3, 4, CV_64FC1);
	calib.getClickViews(cap);
	n = calib.views.pixel.size();
	calib.views.world.assign(n, world);
	calib.getExtrinsics(Tr);

	// extrinsic parameters are now ready to access
	calib.extrinsic_params.R;
	calib.extrinsic_params.t;

	return 0;
}