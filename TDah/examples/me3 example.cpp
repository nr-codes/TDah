#include <iostream>
#include <cv.h>

#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlgs/TrackDot.h"
#include "Cameras/VideoCaptureMe3.h"

#define NDOTS 1
#define ROIW 100
#define ROIH 100

#define NIMGS 100

using namespace cv;

int main()
{
	// all images are Mat objects in OpenCV's C++ documentation
	Mat img;

	/*
	img.create(1024, 1024, CV_8UC1);

	Mat sub = img(Rect(5, 5, 1, 1));

	uchar data[25];
	int x = 52;
	int y = 67;

	Mat cust(5, 5, CV_8UC1, data);
	cust.step = 1024;
	cust.datastart = cust.data - 1024*y - x;
	cust.dataend = cust.datastart + 1024*1024;

	Size wsz;
	Point ofs;
	cust.locateROI(wsz, ofs);

	// data = start + 1024*y + x
	*/

	// choose a video source and tracking algorithm
	VideoCaptureMe3 me3(0); // use the microEnable 3 frame grabber in FastConfig mode
	TrackDot alg(ROIW, ROIH, CV_THRESH_BINARY_INV, 24, 0, ROIW / 2); // use a dot tracking alg

	// setup the tracking system
	Dots dots(NDOTS); // create n dots to track
	Camera cam(me3); // initialize the camera with the video source
	Tracker tracker(alg); // initialize the tracker with the traking alg

	if(!me3.isOpened()) {
		// exit because the camera didn't open properly
		return -1;
	}

	// get initial positions of all NDOTS by user-clicks
	dots.makeAllDotsActive(); // only active dots are updated/modified
	if(NDOTS != tracker.click(cam, dots)) {
		// quit if not all dots have been clicked on
		return -2;
	}

	// track dots across NIMGS images and quit demo
	me3.setRois(dots, Size(ROIW, ROIH), 20e3, 50e3);
	//me3.setRois(dots, Size(1024, 1024), 20e3, 50e3);
	me3.start();
	std::cout << "roi search" << std::endl;
	for(int i = 1; i <= NIMGS; ++i) {
		// grab the next image according to the desired image number
		// and add the dots to the active set
		if(!cam.grab(i, dots)) {
			return -3;
		}

		// track and show the dots
		tracker.track(cam, dots);
		tracker.draw(cam, dots, img);
		if(img.empty()) {
			break;
		}

		imshow("Dots", img);
		waitKey(1);

		// print out location information of active dots
		//std::cout << tracker.str(dots) << std::endl;

		//cv::waitKey(0);

		// TODO enqueue dots!
	}

	waitKey(0);

	return 0;
}