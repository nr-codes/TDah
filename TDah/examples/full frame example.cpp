#include <iostream>
#include <cv.h>

#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlgs/TrackDot.h"

#define NDOTS 3
#define ROIW 15
#define ROIH 15
#define NIMGS 50

using namespace cv;

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

	// get initial positions of all NDOTS by user-clicks
	dots.makeAllDotsActive(); // only active dots are updated/modified
	if(NDOTS != tracker.click(cam, dots)) {
		// quit if not all dots have been clicked on
		return -1;
	}

	// track dots across NIMGS images and quit demo
	for(int i = 1; i <= NIMGS; ++i) {
		// grab the next image and add the dots to the active set
		// note that the parameters are handled internally by the camera 
		// object and are only estimates, since the underlying VideoCapture 
		// does not provide any information at all
		if(!cam.grab(dots)) {
			return -2;
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