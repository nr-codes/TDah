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
#define EXPOSURE 20e3 // set camera exposure time in us
#define FRAME 50e3 // set frame time in us (FRAME >= EXPOSURE + TRANSFER TIME)

using namespace cv;

int increase_priority()
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

	return 0;
}

int main()
{
	// make sure thread/process gets a lot of attention from the OS
	increase_priority();

	// all images are Mat objects in OpenCV's C++ documentation
	Mat img;

	// ** choose a video source and tracking algorithm **
	// use the Silicon Software MicroEnable 3 frame grabber's 
	// FastConfig Applet and Photonfocus TrackCam camera 
	// as the video source
	VideoCaptureMe3 me3(0);
	// use a dot tracking algorithm that binarizes an image 
	// and finds the dot's centroid
	TrackDot alg(ROIW, ROIH, CV_THRESH_BINARY, 247, 0, ROIW / 2);

	// setup the tracking system
	Dots dots(NDOTS); // create n dots to track
	Camera cam(me3); // initialize the camera with the video source
	Tracker tracker(alg); // initialize the tracker with the traking alg

	if(!me3.isOpened()) {
		// exit because the camera didn't open properly
		return -1;
	}

	// ** get initial positions of all NDOTS by user-clicks **
	// change the camera exposure from its default, so the proper threshold 
	// value can be used during the tracking of the dots
	me3.set(CV_CAP_PROP_EXPOSURE, EXPOSURE);
	dots.makeAllDotsActive(); // only active dots are updated/modified
	if(NDOTS != tracker.click(cam, dots)) {
		// quit if not all dots have been clicked on
		return -2;
	}

	// ** initialize the camera's ROIs to the position of the active dots **
	// The first two dots are written to the camera and the rest of the dots
	// are placed in a queue and written to the camera in FIFO order.
	if(NDOTS == 2) {
		// for tracking only two dots (ever) a special case in the
		// queueing order is hit where the sequence of tracking becomes
		// first dot, second dot, first dot, first dot, as oppose to the
		// two dots alternating (i.e., first dot, second dot, first dot,...),
		// this function setup avoids that problem.  Note that if new dots are
		// added they are currently not active until the fifth image
		if(!me3.set2Rois(dots, Size(ROIW, ROIH), EXPOSURE, FRAME)) {
			// couldn't write ROIs to the camera
			return -3;
		}
	}
	else {
		if(!me3.setRois(dots, Size(ROIW, ROIH), EXPOSURE, FRAME)) {
			return -3;
		}
	}
	// calling setRois or set2Rois stops the camera from taking pictures,
	// so need to manually restart the image acquisition process again.
	// In fact, anytime the buffer size, image width, or image height is 
	// changed the image, the image acquisition has to be stopped and the 
	// user must restart the camera.
	if(!me3.start()) {
		// couldn't start image acquisition
		return -4;
	}

	// ** track dots across NIMGS images and quit demo **
	bool toggle = true;
	for(int i = 1; i <= NIMGS; ++i) {
		// output digital signal, useful for measuring cycle time on a 'scope.
		// The output can be found on the TTL trigger board, pin 4.
		me3.set(FG_DIGIO_OUTPUT, toggle);
		toggle = !toggle;

		// grab the next image according to the desired image number
		// and add the dots to the active set
		if(!cam.grab(i, dots)) {
			// image transfer from camera to frame grabber most likely
			// timed out
			break;
		}

		// ** track and show the dots **
		if(!tracker.track(cam, dots)) {
			// not all dots were found
			break;
		}

		// add dots to the ROI queue to eventually be written to the camera
		// This is * very * important.  Dots are removed from the queue, but 
		// the user must add the dots back onto the queue in order for a dot(s)
		// to be tracked in the future.  If the queue is ever empty the last two 
		// dot locations will be where the images will be transferred from, but 
		// not necessary in an alternating order.
		me3.add(dots);

		// ** show the dots **
		// at this point the positions of the active dots are available and the
		// user can send this data through their communication layer
		tracker.draw(cam, dots, img);
		if(img.empty()) {
			return -5;
		}

		imshow("Dots", img);
		waitKey(1);

		// print out location information of active dots
		std::cout << tracker.str(dots) << std::endl;
	}

	return 0;
}