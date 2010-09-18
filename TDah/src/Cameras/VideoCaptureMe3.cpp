#include <utility>
#include <iostream>
#include <queue>

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
#define GET_ROI_TAG(tag) (((tag) >> 16) & 0xf)

#define NROI 2
static int seq[NROI] = {VideoCaptureMe3::ROI_1, VideoCaptureMe3::ROI_0};

using std::queue;
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
	_img_nbr = 1;
	_buffers = 16;
	_trigger = GRABBER_CONTROLLED;
	_tap = FG_CL_DUALTAP_8_BIT;
	_mem = NULL;
	_fg = NULL;

	_r.assign(NROI, std::make_pair(0, 
		Rect(0, 0, FC_MAX_WIDTH, FC_MAX_HEIGHT)));

	_roi.RoiPosX = 0;
	_roi.RoiPosY = 0;
	_roi.RoiWidth = FC_MAX_WIDTH;
	_roi.RoiHeight = FC_MAX_HEIGHT;
	_roi.FrameTimeInMicroSec = 50000.;
	_roi.ExposureInMicroSec = 20000.;

	// never used
	_roi.dComp = 0;
	_roi.dLinlog1 = 0;
	_roi.dLinlog2 = 0;
}

bool VideoCaptureMe3::buffers(int n, int width, int height)
{
	int memsize;

	memsize = n * width * height;
	_buffers = n;
	_roi.RoiWidth = width & MULT_OF_FOUR_MASK; // width must be multiple of 4
	_roi.RoiHeight = height;

	// free and then (re)allocate memory
	if(_mem && Fg_FreeMem(_fg, PORT_A) != FG_OK) {
		me3Err("buffers");
		return false;
	}

	_mem = (uchar*) Fg_AllocMem(_fg, memsize, _buffers, PORT_A);
	if(_mem == NULL) {
		me3Err("buffers");
		return false;
	}

	// update ROIs on camera with new width and height
	// write the ROIs to the camera
	for(int i = 0; i < NROI; ++i) {
		_roi.RoiPosX = _r[i].second.x;
		_roi.RoiPosY = _r[i].second.y;

		_r[i].second.width = _roi.RoiWidth;
		_r[i].second.height = _roi.RoiHeight;
		if(writeParameterSet(_fg, &_roi, i, _r[i].first, true, PORT_A)) {
			me3Err("buffers");
			return false;
		}
	}

	return true;
}

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

bool VideoCaptureMe3::stop()
{
	if(Fg_stopAcquireEx(_fg, PORT_A, _mem, STOP_SYNC) != FG_OK) {
		me3Err("stop");
		return false;
	}

	// clear the queue
	for(; !_q.empty(); _q.pop());

	return true;
}

bool VideoCaptureMe3::roiSequence()
{
	FastConfigSequence fcs;

	// write the ROI sequence
	fcs.mLengthOfSequence = NROI;
	fcs.mRoiPagePointer = seq;
	if(Fg_setParameter(_fg, FG_FASTCONFIG_SEQUENCE, &fcs, PORT_A) != FG_OK) {
		me3Err("roiSequence");
		return false;
	}

	return true;
}

Rect VideoCaptureMe3::calcRoi(const Point2d& pixel) const
{
	int x = cv::saturate_cast<int> (pixel.x);
	int y = cv::saturate_cast<int> (pixel.y);
	int rw = _roi.RoiWidth;
	int rh = _roi.RoiHeight;

	// make sure 0 < x < max width
	x = std::min(std::max(0, x - rw / 2), FC_MAX_WIDTH - rw);
	x &= MULT_OF_FOUR_MASK;

	// make sure 0 < y < max height
	y = std::min(std::max(0, y - rh / 2), FC_MAX_HEIGHT - rh);

	return Rect(x, y, rw, rh);
}

/**
*
* @note writeRoi will map the X location of the dot to the nearest multiple
* of four per the Silicon Software FastConfig documentation
* @note once you commit to adding a dot it cannot be undone
*/

void VideoCaptureMe3::enqueue(const Dots& dots)
{
	int tag;
	Point2d pixel;
	ActiveDots::const_iterator dot;
	ActiveDots& a = dots.activeDots();

	// add active dots to the end of the queue
	for(dot = a.begin(); dot < a.end(); ++dot) {		
		tag = (*dot)->tag();
		pixel = (*dot)->pixel();
		_q.push( std::make_pair( tag, calcRoi( pixel ) ) );
		std::cout << "i added: " << tag << std::endl;
	}
}

bool VideoCaptureMe3::setRois(Dots& dots, cv::Size roi, 
							  double exposure, double fps)
{
	if(!stop()) {
		me3Err("setRois");
		return false;
	}

	// set parameters
	set(CV_CAP_PROP_FRAME_WIDTH, roi.width);
	set(CV_CAP_PROP_FRAME_HEIGHT, roi.height);
	set(CV_CAP_PROP_EXPOSURE, exposure);
	set(CV_CAP_PROP_FPS, fps);

	// add active dots to the end of the queue
	enqueue(dots);

	if(!_q.empty()) {
		// write at most the first NROIs elements to the camera
		int tag;
		for(size_t i = 0; i < NROI; ++i) {
			// get next ROI in sequence
			int slot = seq[i];

			// add element from queue and repeat sequence if _q.size() < NROI
			_r[slot] = _q._Get_container()[i % _q.size()];

			tag = _r[slot].first;
			_roi.RoiPosX = _r[slot].second.x;
			_roi.RoiPosY = _r[slot].second.y;
			if(writeParameterSet(_fg, &_roi, slot, tag, true, PORT_A)) {
				me3Err("setRois");
				return false;
			}
		}

		// remove all elements added to the camera
		for(size_t i = 0; !_q.empty() && i < NROI; ++i, _q.pop());
	}

	return true;
}

bool VideoCaptureMe3::open(int device)
{
	if(_fg) {
		// camera has been allocated before, free the resources
		release();
	}
	
	// initialize the camera and set it to current or default settings
	_fg = Fg_Init(FC_APPLET, PORT_A);
	if(_fg == NULL)
		goto _err;

	if(!set(FG_CAMERA_LINK_CAMTYP, _tap))
		goto _err;

	if(!set(FG_TRIGGERMODE, _trigger))
		goto _err;

	if(!buffers(_buffers, _roi.RoiWidth, _roi.RoiHeight))
		goto _err;

	if(FastConfigInit(PORT_A) != FG_OK)
		goto _err;

	if(!roiSequence())
		goto _err;

	// write the ROIs to the camera
	for(int i = 0; i < NROI; ++i) {
		_roi.RoiPosX = _r[i].second.x;
		_roi.RoiPosY = _r[i].second.y;

		_r[i].first = i;
		_r[i].second.width = _roi.RoiWidth;
		_r[i].second.height = _roi.RoiHeight;
		if(writeParameterSet(_fg, &_roi, i, i, true, PORT_A))
			goto _err;
	}

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

bool VideoCaptureMe3::isOpened() const
{
	return _fg != NULL;
}

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

bool VideoCaptureMe3::grab()
{
	// send software trigger, if necessary
	if(_trigger == ASYNC_SOFTWARE_TRIGGER && 
		!Fg_sendSoftwareTrigger(_fg, PORT_A)) {
		me3Err("grab");
		return false;
	}

	return true;
}

bool VideoCaptureMe3::retrieve(Mat& image, int channel)
{
	int tag;

	// get image data from buffer
	_img_nbr = Fg_getLastPicNumberBlocking(_fg, _img_nbr, PORT_A, TIMEOUT);
	if(_img_nbr < FG_OK) {
		me3Err("retrieve");
		image = Mat();
		return false;
	}

	// get corresponding ROI and copy data
	int slot = _img_nbr % NROI;
	uchar* data = (uchar*) Fg_getImagePtr(_fg, _img_nbr, PORT_A);
	image = Mat(_roi.RoiHeight, _roi.RoiWidth, CV_8UC1, data);
	makeUnsafeMat(image, _r[slot].second.tl());

	// TODO comment out in the future
	tag = _img_nbr;
	if(Fg_getParameter(_fg, FG_IMAGE_TAG, &tag, PORT_A) != FG_OK) {
		me3Err("retrieve");
		return false;
	}
	tag = GET_ROI_TAG(tag); // TODO should be (tag >> (sizeof(int)*8 - 16))
	//std::cout << slot << ":" << tag << ":" << _r[slot].first << std::endl;

	std::cout << "i got: " << tag << std::endl;
	CV_Assert(tag == _r[slot].first);  // TODO delete
	//std::cout << "retrieve: " << _r[slot].second.x << " " << _r[slot].second.y << std::endl;

	if(!_q.empty()) {
		// prepare to write the oldest roi in the queue to the camera
		_r[slot] = _q.front();
		_q.pop();

		tag = _r[slot].first;
		_roi.RoiPosX = _r[slot].second.tl().x;
		_roi.RoiPosY = _r[slot].second.tl().y;

		//std::cout << "adding " << tag << " to slot " << slot << std::endl;

		// write the roi to the camera's free ROI slot
		int do_init = _trigger == ASYNC_SOFTWARE_TRIGGER;
		if(writeParameterSet(_fg, &_roi, slot, tag, do_init, PORT_A)) {
			me3Err("retrieve");
			return false;
		}
	}

	return true;
}

double VideoCaptureMe3::nextDot()
{
	static double next_dot = 0;

	if(next_dot < NROI) {
		++next_dot;
		return _r[_img_nbr % NROI].first;
	}

	next_dot = 0;
	return  NO_MORE_DOTS;
}

bool VideoCaptureMe3::set(int prop, double value)
{
	int rc;

	switch(prop) {
		case CV_CAP_PROP_POS_FRAMES:
			// set the next desired image number
			_img_nbr = static_cast<int> (value);
			return true;

		case CV_CAP_PROP_FRAME_WIDTH:
			// update width and resize memory
			return buffers(_buffers, static_cast<int> (value), _roi.RoiHeight);

		case CV_CAP_PROP_FRAME_HEIGHT:
			// update height and resize memory
			return buffers(_buffers, _roi.RoiWidth, static_cast<int> (value));

		case CV_CAP_PROP_FRAME_COUNT:
			// update buffers and resize memory
			return buffers(static_cast<int> (value), 
				_roi.RoiWidth, _roi.RoiHeight);

		case CV_CAP_PROP_FPS:
			// set the frame time
			_roi.FrameTimeInMicroSec = 1e6 / value;
			return true;

		case CV_CAP_PROP_EXPOSURE:
			_roi.ExposureInMicroSec = value;
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
			
			rc = static_cast<double> (_roi.RoiHeight != FC_MAX_HEIGHT ||
				_roi.RoiWidth != FC_MAX_WIDTH);
			break;

		case CV_CAP_PROP_POS_MSEC:
			ts = _img_nbr;
			if(Fg_getParameter(_fg, FG_TIMESTAMP_LONG, &ts, PORT_A) != FG_OK) {
				me3Err("get");
				rc = 0;
			}
			else {
				rc = static_cast<double> (ts * 1e-3);
			}
			break;

		case CV_CAP_PROP_POS_FRAMES:
			// get the current image number
			rc = static_cast<double> (_img_nbr);
			break;

		case CV_CAP_PROP_FRAME_WIDTH:
			// get the current width
			rc = static_cast<double> (_roi.RoiWidth);
			break;

		case CV_CAP_PROP_FRAME_HEIGHT:
			// get the current height
			rc = static_cast<double> (_roi.RoiHeight);
			break;

		case CV_CAP_PROP_FRAME_COUNT:
			// get the current number of buffers
			rc = static_cast<double> (_buffers);
			break;

		case CV_CAP_PROP_FPS:
			// set the frame time
			rc = 1e6 / _roi.FrameTimeInMicroSec;
			break;

		case CV_CAP_PROP_EXPOSURE:
			rc = _roi.ExposureInMicroSec;
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