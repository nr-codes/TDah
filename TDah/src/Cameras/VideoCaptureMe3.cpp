#include <iostream>

#include "Dots.h"
#include "Cameras/VideoCaptureMe3.h"
#include "TrackingAlgs/TrackDot.h"

#if defined(WIN32) && defined(_WIN32)
	#define FC_APPLET "FastConfig.dll"
#endif

#define TIMEOUT 2
#define FC_MAX_WIDTH 1024
#define FC_MAX_HEIGHT 1024
#define MULT_OF_FOUR_MASK (-4)

#define NROI 2
static int seq[NROI] = {VideoCaptureMe3::ROI1, VideoCaptureMe3::ROI0};

using std::deque;
using std::string;

using cv::Mat;
using cv::Rect;
using cv::Point;
using cv::Point2d;

VideoCaptureMe3::VideoCaptureMe3()
{
	fastConfigDefaults();
}

VideoCaptureMe3::VideoCaptureMe3(const std::string& filename)
{
	fastConfigDefaults();
	open(filename);
}

VideoCaptureMe3::VideoCaptureMe3(int device)
{
	fastConfigDefaults();
	open(device);
}

VideoCaptureMe3::~VideoCaptureMe3()
{
	release();
}

double VideoCaptureMe3::nextDot()
{
	static double next_dot = 0;

	if(next_dot < NROI) {
		++next_dot;
		return _roi[_img_nbr % NROI].first;
	}

	next_dot = 0;
	return  BAD_TAG;
}

/**
* Returns the tag set by calls to writeRoi, which is located in
* the upper 16-bits of the image info tag, see Silicon Software 
* documentation (FastConfig and SDK) for more info.
*
* @param[in] img_info the output value of Fg_getParameter(...)
* with FG_IMAGE_TAG set as the nParameter argument.
*/

int VideoCaptureMe3::getTag(int img_tag)
{
	// don't know how this will work when int is not 32-bits
	// potential fix might be: (sizeof(int)*8 - 16)
	CV_DbgAssert(sizeof(int) == 4);
	return (signed) (((unsigned) img_tag) >> 16);
}

void VideoCaptureMe3::makeSafeMat(Mat& mat)
{
	mat = Mat(mat.rows, mat.cols, mat.type(), mat.data);
}

void VideoCaptureMe3::makeUnsafeMat(Mat& mat, Point& offset)
{
	mat.step = FC_MAX_WIDTH;
	mat.datastart = mat.data - FC_MAX_WIDTH * offset.y - offset.x;
	mat.dataend = mat.datastart + FC_MAX_WIDTH * FC_MAX_HEIGHT;
}

void VideoCaptureMe3::me3Err(string msg)
{
	std::cout << "me3::" << msg << ": (" << 
		Fg_getLastErrorNumber(_fg) << ") " <<
		Fg_getLastErrorDescription(_fg) << std::endl;
}

void VideoCaptureMe3::fastConfigDefaults()
{
	// setup parameters
	_img_nbr = 1;
	_buffers = 16;
	_trigger = GRABBER_CONTROLLED;
	_tap = FG_CL_DUALTAP_8_BIT;
	_mem = NULL;
	_fg = NULL;

	// allocate data structures
	FC_ParameterSet roi;
	roi.RoiPosX = 0;
	roi.RoiPosY = 0;
	roi.RoiWidth = FC_MAX_WIDTH;
	roi.RoiHeight = FC_MAX_HEIGHT;
	roi.FrameTimeInMicroSec = 50000.;
	roi.ExposureInMicroSec = 20000.;
	roi.dComp = 0; // never used
	roi.dLinlog1 = 0; // never used
	roi.dLinlog2 = 0; // never used
	_roi.assign(NROI, std::make_pair(0, roi));

	_roi_in_buffer.assign(_buffers, std::make_pair(BAD_TAG, Rect(0, 0, 0, 0)));
	_q.clear();
}

bool VideoCaptureMe3::buffers(int n, int width, int height)
{
	int memsize;

	// free memory, note that image acquisition is stopped by Fg_FreeMem(...)
	if(_mem && Fg_FreeMem(_fg, PORT_A) != FG_OK) {
		me3Err("buffers");
		return false;
	}

	// (re)allocate memory
	memsize = n * width * height;
	_mem = (uchar*) Fg_AllocMem(_fg, memsize, n, PORT_A);
	if(_mem == NULL) {
		me3Err("buffers");
		return false;
	}

	// set new parameters
	_buffers = n;
	_roi_in_buffer.assign(_buffers, std::make_pair(BAD_TAG, Rect(0, 0, 0, 0)));

	bool all_written = true;
	for(size_t i = 0; i < _roi.size(); ++i) {
		// width must be multiple of 4 (see Silicon Software FastConfig doc)
		_roi[i].second.RoiWidth = width & MULT_OF_FOUR_MASK;
		_roi[i].second.RoiHeight = height;

		if(!writeRoi(i)) {
			// memory was successfully allocated, but an ROI
			// was not correctly written, so finish updating
			// parameters and then return an error
			all_written = false;
			me3Err("buffers");
		}
	}

	return all_written;
}

/**
* Starts the image acquisition with the image number starting at 1
*
* @param[in] n the number of image to take, the special value,
* GRAB_INFINITE, will grab images until the image acquisition is
* stopped.  Functions that stop the acquisition are stop(...), 
* release(...), setRois(...), and buffers(...).
*/
bool VideoCaptureMe3::start(int n)
{
	// start acquiring
	if(Fg_Acquire(_fg, PORT_A, n) != FG_OK) {
		me3Err("acquire");
		return false;
	}

	_img_nbr = 1;
	return true;
}

/**
* Stops the image acquisition process, so no more new images 
* are taken.
*/

bool VideoCaptureMe3::stop()
{
	// Fg_stopAcquire crashes the program, so using Fg_stopAcquireEx
	if(Fg_stopAcquireEx(_fg, PORT_A, _mem, STOP_SYNC) != FG_OK) {
		me3Err("stop");
		return false;
	}

	return true;
}

/**
* Writes the hard-coded ROI sequence as specified by the private variable,
* seq, to the frame grabber/camera.  The length of the sequence used is NROI.
*/

bool VideoCaptureMe3::roiSequence()
{
	FastConfigSequence fcs;

	// write the ROI sequence, which is hard-coded at the top of this file
	fcs.mLengthOfSequence = NROI;
	fcs.mRoiPagePointer = seq;
	if(Fg_setParameter(_fg, FG_FASTCONFIG_SEQUENCE, &fcs, PORT_A) != FG_OK) {
		me3Err("roiSequence");
		return false;
	}

	return true;
}

/**
* Calculates a valid window around the pixel position.  If the window
* is fully inside the image frame, then the ROI will (almost) be centered 
* around the pixel's location.  Because of limitations of the FastConfig
* interface, x locations must be multiples of four (4) in order for the
* camera to accept the ROI, hence the pixel may be slightly off-center.  If
* the window is not fully inside the image frame, then the placement of the
* ROI will be adjusted accordingly, so it fits within the image frame.
*
* @param[in] pixel the (x,y) location of the pixel to center the ROI around
*
* @note For more information on FastConfig limitations read the Silicon 
* Software documentation on FastConfig.
*
* @attention this function assumes all ROIs use the same width and height
*/

Rect VideoCaptureMe3::calcRoi(const Point2d& pixel) const
{
	int x = cv::saturate_cast<int> (pixel.x);
	int y = cv::saturate_cast<int> (pixel.y);
	int rw = _roi[0].second.RoiWidth;
	int rh = _roi[0].second.RoiHeight;

	// make sure 0 < x < max width
	x = std::min(std::max(0, x - rw / 2), FC_MAX_WIDTH - rw);
	x &= MULT_OF_FOUR_MASK;

	// make sure 0 < y < max height
	y = std::min(std::max(0, y - rh / 2), FC_MAX_HEIGHT - rh);

	return Rect(x, y, rw, rh);
}

/**
* Adds the active dots to the end of an internally maintained queue.
* Every time grab() is called a dot is popped from the front of the
* queue and written to the camera.  This function allows the user to
* track more than eight (8) dots at a time, since the FastConfig interface
* only has eight ROI slots that can be used.
*
* @param[in] dots the object containing the active dots to add to the end
* of the queue.
*/

void VideoCaptureMe3::add(const Dots& dots)
{
	int tag;
	Point2d pixel;
	ActiveDots::const_iterator dot;

	// add active dots to the end of the queue
	ActiveDots& a = dots.activeDots();
	for(dot = a.begin(); dot < a.end(); ++dot) {		
		tag = (*dot)->tag();
		pixel = (*dot)->pixel();
		_q.push_back( std::make_pair( tag, calcRoi( pixel ) ) );
	}
}

/**
* Removes the dot associated with tag from the queue
*
* @param[in] tag the tag value to remove, if this parameters has
* the special value, VideoCaptureMe3::REMOVE_ALL, then all dots 
* are removed from the queue.
*/

void VideoCaptureMe3::remove(int tag)
{
	if(tag == REMOVE_ALL) {
		_q.clear();
	}
	else {
		std::deque< std::pair< int, cv::Rect> >::const_iterator it;
		for(it = _q.begin(); it < _q.end() && (*it).first != tag; ++it);
		if(it != _q.end())
			_q.erase(it);
	}
}

/**
* Writes the region of interest stored in the class's local ROI cache
* to the camera.
*
* @param[in] slot the ROI slot that the data will be written to
* @return true, if the parameters were written successfully, false otherwise.
*/

bool VideoCaptureMe3::writeRoi(int slot)
{
	int tag;
	int do_init;
	FC_ParameterSet* roi;

	// send a sync signal to the frame grabber, see Silicon Software 
	// FastConfig documentation for writeParameterSet(...) for more info
	do_init = _trigger == ASYNC_SOFTWARE_TRIGGER || _img_nbr == 1;

	// write the ROI to the camera
	tag = _roi[slot].first;
	roi = &_roi[slot].second;
	if(writeParameterSet(_fg, roi, slot, tag, do_init, PORT_A)) {
		me3Err("writeRoi");
		return false;
	}

	return true;
}

/**
* Stops all image acquisition and sets the ROIs to match the pixel 
* positions specified in the active dots, as well as the ROI window size, 
* exposure time, and frame rate.  The dots will be written to the camera 
* in the order that they appear in the active set.  The user must restart
* the image acquisition by calling start(...) after this function returns.
*
* @param[in] dots the (active) dots to write to the camera
* @param[in] roi the size of the region-of-interest window
* @param[in] exposure the exposure time in microseconds
* @param[in] fps the frames per second to acquire the images at
*
* @return true, if every dot's position has been successfully
* written to the camera's ROI.  False, otherwise.
*
* @note fps has no effect if the trigger signal is through software
* or an external trigger source.
*/

bool VideoCaptureMe3::setRois(Dots& dots, cv::Size roi, 
							  double exposure, double fps)
{
	if(!stop()) {
		me3Err("setRois");
		return false;
	}

	// set parameters
	set(CV_CAP_PROP_EXPOSURE, exposure);
	set(CV_CAP_PROP_FPS, fps);
	buffers(_buffers, roi.width, roi.height);

	// add active dots to the end of the queue
	add(dots);

	if(!_q.empty()) {
		// write at most the first NROIs elements to the camera
		for(size_t i = 0; i < NROI; ++i) {
			// get next ROI in sequence
			int slot = seq[i];

			// write element from queue and repeat sequence if _q.size() < NROI
			_roi[slot].first = _q[i % _q.size()].first;
			_roi[slot].second.RoiPosX = _q[i % _q.size()].second.x;
			_roi[slot].second.RoiPosY = _q[i % _q.size()].second.y;

			if(!writeRoi(slot)) {
				me3Err("setRois");
				return false;
			}
		}

		// remove all elements added to the camera
		for(size_t i = 0; !_q.empty() && i < NROI; ++i, _q.pop_front());
	}

	return true;
}

/*
void VideoCaptureMe3::updateRoiBuffer()
{
#if _DEBUG09
	int tag = _img_nbr;
	if(Fg_getParameter(_fg, FG_IMAGE_TAG, &tag, PORT_A) != FG_OK) {
		me3Err("retrieve");
		return false;
	}

	tag = getTag(tag);
	CV_Assert(tag == _roi_in_buffer[k].first);
#endif
}
*/

bool VideoCaptureMe3::updateRoi()
{
	// setup the buffer and slot indices
	int slot = _img_nbr % NROI;
	int buf = (_img_nbr - 1) % _buffers;

	// record which ROI is in the frame grabber buffer
	_roi_in_buffer[buf].first = _roi[slot].first;
	_roi_in_buffer[buf].second.x = _roi[slot].second.RoiPosX;
	_roi_in_buffer[buf].second.y = _roi[slot].second.RoiPosY;
	_roi_in_buffer[buf].second.width = _roi[slot].second.RoiWidth;
	_roi_in_buffer[buf].second.height = _roi[slot].second.RoiHeight;

	// write new ROI to camera
	if(!_q.empty()) {
		// prepare to write the oldest roi in the queue to the camera
		_roi[slot].first = _q.front().first;
		_roi[slot].second.RoiPosX = _q.front().second.x;
		_roi[slot].second.RoiPosY = _q.front().second.y;

		// write the roi to the camera
		if(!writeRoi(slot)) {
			me3Err("grab");
			return false;
		}

		// the ROI has been written, remove it from the queue
		_q.pop_front();
	}

	return true;
}

/**
* Initializes the Silicon Software frame grabber and FastConfig interface
* using either the default parameters, if this is a new instance of the object,
* or resuses previously set parameters for multiple calls to this function.  If
* the camera has previously been opened, it is closed before reinitializing.
*
* @param[in] device the port to read from
*
* @note The device parameter is ignored.  In order to use FastConfig, PORTA, 
* must be used.  See the Silicon Software FastConfig documentation for more 
* information.
*
* @attention this function assumes all ROIs use the same width and height
*/

bool VideoCaptureMe3::open(int device)
{
	if(_fg) {
		// camera has been allocated before, free the resources
		release();
	}
	
	// initialize the camera and FastConfig interface
	_fg = Fg_Init(FC_APPLET, PORT_A);
	if(_fg == NULL)
		goto _err;

	if(FastConfigInit(PORT_A) != FG_OK)
		goto _err;

	// set camera to current settings
	if(!set(FG_CAMERA_LINK_CAMTYP, _tap))
		goto _err;

	if(!set(FG_TRIGGERMODE, _trigger))
		goto _err;

	// write the hard-coded ROI sequence
	if(!roiSequence())
		goto _err;

	// allocate the buffers (which also writes the ROIs to the camera)
	if(!buffers(_buffers, _roi[0].second.RoiWidth, _roi[0].second.RoiHeight))
		goto _err;

	// start the camera
	if(!start()) {
		goto _err;
	}

	return true;

_err:
	me3Err("open");
	release();
	return false;
}

bool VideoCaptureMe3::open(const string& filename)
{
	_fg = NULL;
	return false;
}

/**
* Returns true if the camera has been successfully opened
*
* @return true, if the camera is open and false otherwise
*/

bool VideoCaptureMe3::isOpened() const
{
	return _fg != NULL;
}

/*
* Frees all of the frame grabber's resources, closes the FastConfig
* communication channel, and turns off the Exsync signal.  The function
* also resets all internal parameters to its original default settings
*
* @see fastConfigDefaults
*/

void VideoCaptureMe3::release()
{
	if(_fg == NULL) {
		return;
	}

	// turn off external sync signal
	if(Fg_setExsync(_fg, FG_OFF, PORT_A) != FG_OK) {
		me3Err("release");
	}

	// release fastconfig communication channel
	if(FastConfigFree(PORT_A) != FG_OK) {
		me3Err("release");
	}

	// stop acquiring and free memory
	if(Fg_FreeGrabber(_fg) != FG_OK) {
		me3Err("release");
	}

	fastConfigDefaults();
}

/** 
* Grabs an image from the camera and waits until the image has been
* transferred to the frame grabber.  If an image has been successfully
* transferred, then the next ROI in the queue is written to the camera
* and removed from the queue.  If the user wants to specify which image
* to grab, then a call to the set function with prop = CV_CAP_PROP_POS_FRAMES
* prior to calling this function will grab the >= n-th image specified by 
* value.  The user should call the corresponding get function to see which
* image number was actually transferred.
*
* @return true, if an image has been successfully transferred and a new ROI
* has been written to the camera, if there are any ROIs pending to be written.
* False, otherwise.
* 
* @see set, get
*/

bool VideoCaptureMe3::grab()
{
	// send software trigger, if necessary
	if(_trigger == ASYNC_SOFTWARE_TRIGGER && 
		!Fg_sendSoftwareTrigger(_fg, PORT_A)) {
		me3Err("grab");
		return false;
	}

	// grab the desired image and update what image number the camera is at
	_img_nbr = Fg_getLastPicNumberBlocking(_fg, _img_nbr, PORT_A, TIMEOUT);
	if(_img_nbr < FG_OK) {
		me3Err("grab");
		return false;
	}

	// new image grabbed, so update ROI buffer 
	// and write next ROI in queue to free slot
	return updateRoi();
}

bool VideoCaptureMe3::retrieve(Mat& image, int channel)
{
	// verify ROI in buffer is still valid, otherwise it should
	// be the ROI stored on the camera

	// get corresponding ROI and copy data
	int k = (_img_nbr - 1) % _buffers;
	Rect& r = _roi_in_buffer[k].second;
	uchar* data = (uchar*) Fg_getImagePtr(_fg, _img_nbr, PORT_A);

	// create the matrix and set it up so locateROI works correctly
	// warning: this matrix must only be accessed within the data field
	image = Mat(r.height, r.width, CV_8UC1, data);
	makeUnsafeMat(image, r.tl());

	return true;
}

bool VideoCaptureMe3::set(int prop, double value)
{
	int rc;

	switch(prop) {
		case CV_CAP_PROP_POS_FRAMES:
			// set the next desired image number
			_img_nbr = static_cast<int> (value);
			return true;

		case CV_CAP_PROP_FRAME_WIDTH: // assumes all heights are the same
			// update width and resize memory
			return buffers(_buffers, static_cast<int> (value), 
				_roi[0].second.RoiHeight);

		case CV_CAP_PROP_FRAME_HEIGHT: // assumes all widths are the same
			// update height and resize memory
			return buffers(_buffers, _roi[0].second.RoiWidth, 
				static_cast<int> (value));

		case CV_CAP_PROP_FRAME_COUNT: // assumes all widths/heights are equal
			// update buffers and resize memory
			return buffers(static_cast<int> (value), 
				_roi[0].second.RoiWidth, _roi[0].second.RoiHeight);

		case CV_CAP_PROP_FPS:
			// set the frame time
			for(size_t i = 0; i < _roi.size(); ++i)
				_roi[i].second.FrameTimeInMicroSec = 1e6 / value;
			return true;

		case CV_CAP_PROP_EXPOSURE:
			// set the exposure time
			for(size_t i = 0; i < _roi.size(); ++i)
				_roi[i].second.ExposureInMicroSec = value;
			return true;

		case FG_TRIGGERMODE:
			// set the trigger mode (see Silicon Software documentation)
			_trigger = static_cast<int> (value);
			if(Fg_setParameter(_fg, FG_TRIGGERMODE, &_trigger, PORT_A)) {
				me3Err("set");
				return false;
			}

			if(_trigger != FREE_RUN) {
				// enable the exsync pin for non-free running modes
				if(Fg_setExsync(_fg, FG_ON, PORT_A) != FG_OK) {
					me3Err("set");
					return false;
				}
			}
			return true;

		case FG_TRIGGERINSRC:
			// enable the TTL Trigger Pin 12 for external triggering
			rc = TRGINSRC_1;
			if(Fg_setParameter(_fg, FG_TRIGGERINSRC, &rc, PORT_A) != FG_OK) {
				me3Err("set");
				return false;
			}
			return true;

		case FG_CAMERA_LINK_CAMTYP:
			// set either dual or single tap data transfers
			_tap = static_cast<int> (value);
			if(Fg_setParameter(_fg, FG_CAMERA_LINK_CAMTYP, &_tap, PORT_A)) {
				me3Err("set");
				return false;
			}
			return true;
	}

	return false;
}

double VideoCaptureMe3::get(int prop)
{
	uint64 ts;
	double rc = 0;

	switch(prop) {
		case TDAH_PROP_NEXT_DOT:
			rc = nextDot();
			break;

		case TDAH_PROP_IS_ROI:
			
			rc = static_cast<double> 
				(_roi[0].second.RoiHeight != FC_MAX_HEIGHT ||
				_roi[0].second.RoiWidth != FC_MAX_WIDTH);
			break;

		case CV_CAP_PROP_POS_MSEC: // returns timestamp in microseconds
			ts = _img_nbr;
			if(Fg_getParameter(_fg, FG_TIMESTAMP_LONG, &ts, PORT_A) != FG_OK) {
				me3Err("get");
				rc = 0;
			}
			else {
				rc = static_cast<double> (ts);
			}
			break;

		case CV_CAP_PROP_POS_FRAMES:
			// get the current image number
			rc = static_cast<double> (_img_nbr);
			break;

		case CV_CAP_PROP_FRAME_WIDTH:
			// get the current width (assumes all ROIs use the same width)
			rc = static_cast<double> (_roi[0].second.RoiWidth);
			break;

		case CV_CAP_PROP_FRAME_HEIGHT:
			// get the current height (assumes all ROIs use the same height)
			rc = static_cast<double> (_roi[0].second.RoiHeight);
			break;

		case CV_CAP_PROP_FRAME_COUNT:
			// get the current number of buffers
			rc = static_cast<double> (_buffers);
			break;

		case CV_CAP_PROP_FPS: // assumes same fps for all ROIs
			// get the frame time
			rc = 1e6 / _roi[0].second.FrameTimeInMicroSec;
			break;

		case CV_CAP_PROP_EXPOSURE: // assumes same exposure for all ROIs
			// get the exposure time in microseconds
			rc = _roi[0].second.ExposureInMicroSec;
			break;

		case FG_TRIGGERMODE:
			// get the trigger mode (see Silicon Software documentation)
			rc = static_cast<double> (_trigger);
			break;

		case FG_TRIGGERINSRC:
			// get the TTL Trigger used for external triggering
			rc = static_cast<double> (TRGINSRC_1);
			break;

		case FG_CAMERA_LINK_CAMTYP:
			// get the tap data transfer type
			rc = static_cast<double> (_tap);
			break;
	}

	return rc;
}