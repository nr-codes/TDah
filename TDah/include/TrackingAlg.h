#ifndef _TRACKING_ALG_H_
#define _TRACKING_ALG_H_

#include <vector>
#include <string>
#include <cv.h>
#include "_common.h"

/**
* A class for passing to a Tracker object.
*/

class TrackingAlg
{
public:
	TrackingAlg();

	/** @brief the destructor for this class */
	virtual ~TrackingAlg();

	/** @brief finds a dot in an image */
	virtual bool find(const cv::Mat& img, const Dot& dot, cv::Point2d& new_loc);

	/** @brief draws the current location of a dot */
	virtual void draw(const cv::Mat& src, const Dot& dot, cv::Mat& dst);

	/** @brief returns the name of the window that will listen for events */
	virtual const std::string& clickingWindow();

protected:
	std::string _click_window;
};

#endif /* _TRACKING_ALG_H_ */