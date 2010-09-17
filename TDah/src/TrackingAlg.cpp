#include "TrackingAlg.h"

using std::vector;
using std::string;
using cv::Mat;
using cv::Point2d;
using cv::Size;

TrackingAlg::TrackingAlg()
{
	_click_window = "Warning: No Tracking Algorithm Present";
}

TrackingAlg::~TrackingAlg()
{
	return;
}

/**
* Searches for a dot in the current image based on the previous known location
* of the dot and, if the dot is found, updates the new location.
*
* @param[in] img the current image
* @param[in] prev_loc the previous location of the dot
* @param[out] new_loc the new location of the dot
* @return true, if the dot is found and <code> new_loc </code> contains the new
* location, false otherwise.
*/

bool TrackingAlg::find(const Mat& img, Size max_resolution, const Dot& dot, 
					   Point2d& new_loc)
{
	return false;
}

/**
* Draws the current position of the dot based on the source image.
*
* @param[in] src the source image
* @param[in] dot the dot to draw
* @param[out] dst the destination image to draw on
*/

void TrackingAlg::draw(const Mat& src, const Dot& dot, Mat& dst)
{
	dst = src;
}

/**
* Returns the name of the clicking window that will be displayed to the user
* when Tracker::click(...) is called.
*
* @note This window, for example, may have an additional trackbar slider, 
* so that descendents of TrackingAlg can initialize their specific 
* parameters. See TrackDot
*
* @return the name of the clicking window
*/
const string& TrackingAlg::clickingWindow()
{
	return _click_window;
}