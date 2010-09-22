#include <iostream>
#include <cv.h>

#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlgs/TrackDot.h"
#include "Cameras/VideoCaptureMe3.h"

#define NDOTS 2
#define ROIW 40
#define ROIH 40

#define NIMGS 1000

using namespace cv;

int main()
{
	// change priority and thread class  -- WINDOWS only --
	int rc = SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS );
	if(rc == FALSE) {
      return GetLastError();
	}

	rc = SetThreadPriority(GetCurrentThread(), HIGH_PRIORITY_CLASS);
	if(rc == FALSE) {
      return GetLastError();
   }

	// all images are Mat objects in OpenCV's C++ documentation
	Mat img;

	// choose a video source and tracking algorithm
	VideoCaptureMe3 me3(0); // use the microEnable 3 frame grabber in FastConfig mode
	TrackDot alg(ROIW, ROIH, CV_THRESH_BINARY, 247, 0, ROIW / 2); // use a dot tracking alg

	// setup the tracking system
	Dots dots(NDOTS); // create n dots to track
	Camera cam(me3); // initialize the camera with the video source
	Tracker tracker(alg); // initialize the tracker with the traking alg

	if(!me3.isOpened()) {
		// exit because the camera didn't open properly
		return -1;
	}

	me3.set(CV_CAP_PROP_EXPOSURE, .10e3);

	// get initial positions of all NDOTS by user-clicks
	dots.makeAllDotsActive(); // only active dots are updated/modified
	if(NDOTS != tracker.click(cam, dots)) {
		// quit if not all dots have been clicked on
		return -2;
	}

	// track dots across NIMGS images and quit demo
	if(NDOTS == 2) {
		me3.set2Rois(dots, Size(ROIW, ROIH), .10e3, .3e3);
	}
	else {
		me3.setRois(dots, Size(ROIW, ROIH), .10e3, .3e3);
	}

	Util timer("main", NIMGS);
	me3.start();
	bool toggle = true;
	for(int i = 1; i <= NIMGS; ++i) {
		timer.start();
		me3.set(FG_DIGIO_OUTPUT, toggle);
		toggle = !toggle;

		// grab the next image according to the desired image number
		// and add the dots to the active set
		if(!cam.grab(i, dots)) {
			break;
		}

		// track and show the dots
		if(!tracker.track(cam, dots)) {
			// not all dots were found
			break;
		}

		// queue up the ROIs to be written to the camera
		me3.add(dots);

/*
		tracker.draw(cam, dots, img);
		if(img.empty()) {
			return -5;
		}

		imshow("Dots", img);
		waitKey(1);
*/

		// print out location information of active dots
		//std::cout << tracker.str(dots) << std::endl;
		timer.stop();
	}

	timer.printResults();
	me3.retrieve(img);
	imshow("img", img);
	waitKey();
	return 0;
}