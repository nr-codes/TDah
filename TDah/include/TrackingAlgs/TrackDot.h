#ifndef _TRACKDOT_H_
#define _TRACKDOT_H_

#include "TrackingAlg.h"

/**
* @brief The base class for tracking dots
*
* A Tracker tracks the set of active dots in an image, updating
* the position of the active dots according to an implementation-
* specific algorithm.  This class is an abstract class.
*/

class TrackDot : public TrackingAlg
{
public:
	TrackDot(int roi_width, int roi_height, int threshold_type);
	TrackDot(int roi_width, int roi_height, int threshold, int threshold_type, 
		double min_radius, double max_radius);

	/** @brief the destructor for this class */
	virtual ~TrackDot();

	/** @brief finds a dot in an image */
	virtual bool find(const cv::Mat& img, const Dot& dot, cv::Point2d& new_loc);

	/** @brief draws a dot in an image */
	virtual void draw(const cv::Mat& src, const Dot& dot, cv::Mat& dst);

	/** @brief returns the name of the clicking window */
	virtual const std::string& clickingWindow();

	/** @brief sets all dot tracking parameters */
	void set(int roi_width, int roi_height, int threshold, int threshold_type, 
		double min_radius, double max_radius);

	/** @brief a wrapper function that accepts 1-channel or 3-channel images */
	void threshold(const cv::Mat& src, const cv::Rect roi, cv::Mat& dst) const;

private:
	/** @brief the threshold value to use */
	int _thr;
	/** @brief the threshold type (see OpenCV's threshold documentation) */
	int _thr_type;
	/** @brief the width of the tracking rectangle */
	int _rw;
	/** @brief the height of the tracking rectangle */
	int _rh;
	/** @brief the minimum size radius to track */
	double _minr;
	/** @brief the maximum size radius to track */
	double _maxr;
	/** @brief the name of the trackbar window */
	std::string _trackbar_window;

	/** @brief calculates a valid tracking rectangle inside the image */
	cv::Rect calcRoi(const cv::Point2d& pixel, const cv::Size& img_size) const;
};

#endif /* _TRACKDOT_H_ */