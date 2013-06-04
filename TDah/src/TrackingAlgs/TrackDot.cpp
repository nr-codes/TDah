#include <highgui.h>
#include "Dot.h"
#include "TrackingAlgs/TrackDot.h"


#include <iostream>
#include <conio.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

#define WHITE 255
#define LOC_COLOR Scalar(255, 0, 0)
#define TAG_COLOR Scalar(0, 165, 255)

using std::stringstream;
using std::vector;
using std::string;
using cv::Mat;
using cv::Mat_;
using cv::Point2d;
using cv::Point2f;
using cv::Point;
using cv::Rect;
using cv::Size;
using cv::Scalar;

TrackDot::TrackDot(int roi_width, int roi_height, int threshold_type)
{
	set(roi_width, roi_height, 0, threshold_type, 0, 
		std::min(roi_width, roi_height) / 2);
}

TrackDot::TrackDot(int roi_width, int roi_height, int threshold_type, 
				   int threshold, double min_radius, double max_radius)
{
	set(roi_width, roi_height, threshold, 
		threshold_type, min_radius, max_radius);
}

/** @brief the destructor for this class */
TrackDot::~TrackDot()
{

}

void TrackDot::set(int roi_width, int roi_height, int threshold, 
				   int threshold_type, double min_radius, double max_radius)
{
	_click_window = "TrackDot";
	_trackbar_window = "Threshold";
	_thr = threshold;
	_thr_type = threshold_type;
	_rw = roi_width;
	_rh = roi_height;
	_minr = min_radius;
	_maxr = max_radius;
}

const string& TrackDot::clickingWindow()
{
	cv::namedWindow(_click_window);
	cv::createTrackbar(_trackbar_window, _click_window, &_thr, WHITE);
	return _click_window;
}

Rect TrackDot::calcRoi(const Point2d& pixel, const Size& img_size) const
{
	int x = cv::saturate_cast<int> (pixel.x);
	int y = cv::saturate_cast<int> (pixel.y);
	//std::cout << "help " << std::endl;
	//std::cout << img_size.width << " " << _rw << std::endl;
	// avoid negative x and y values
	CV_Assert(img_size.width >= _rw && img_size.height >= _rh);
	//CV_Assert(img_size.height >= _rh);

	// make sure 0 < x < max width
	x = std::min(std::max(0, x - _rw / 2), img_size.width - _rw);

	// make sure 0 < y < max height
	y = std::min(std::max(0, y - _rh / 2), img_size.height - _rh);

	return Rect(x, y, _rw, _rh);
}

/**
* Thresholds the source image and places it in the destination image.
* The function will accept either gray scale images (CV_8UC1) or BGR
* color images (CV_8UC3).  The output image will be gray scale (CV_8UC1).
*
* @param[in] src the source image
* @param[in] roi the region-of-interest to grayscale
* @param[out] dst the destination image
*/

void TrackDot::threshold(const Mat& src, const Rect roi, Mat& dst) const
{
	//Mat tdst;
	// get 1-ch image subregion
	if(src.type() == CV_8UC1) {
		src(roi).copyTo(dst); // TODO do i really want to copy src into dst?
	}
	else if(src.type() == CV_8UC3) {
		dst.create(_rh, _rw, CV_8UC1);
		cv::cvtColor(src(roi), dst, CV_BGR2GRAY);
	}
	else {
		// raise an error, because image must be BGR, or grayscale
		CV_Assert(src.type() == CV_8UC1 || src.type() == CV_8UC3);
	}

	// binarize image
	cv::threshold(dst, dst, _thr, WHITE, _thr_type);
	//cv::adaptiveThreshold(dst,tdst,WHITE,0,_thr_type,51,5);
	//dst=tdst;
}

/**
* Draws a threshold patch inside the region-of-interest in the destination 
* image centered at the dot's current location.
*
* @param[in] src the source image
* @param[in] dot the dot to draw
* @param[out] dst the destination image
*/
void TrackDot::draw(const Mat& src, const Dot& dot, Mat& dst)
{
	Mat gr;
	stringstream ss;
	Rect roi = calcRoi(dot.pixel(), src.size());

	// binarize image
	threshold(src, roi, gr);

	// draw dot location and ROI rectangle
	dst.create(src.rows, src.cols, CV_8UC3);
	cv::cvtColor(gr, dst(roi), CV_GRAY2BGR);
	cv::rectangle(dst, roi.tl(), roi.br(), LOC_COLOR);

	// draw the tag number at the pixel location
	ss << dot.tag();
	//cv::putText(dst, ss.str(), dot.pixel(), 
	cv::putText(dst, ss.str(), (roi.tl() + roi.br()) * 0.5, 
		CV_FONT_HERSHEY_PLAIN, 1, TAG_COLOR);
}

/**
* finds a dot in an image by searching for the best fit circle on a subset
* of boundary points of a binary image.  If the subset of boundary points form
* a circle with radius between the user-specified min and max radii, then the 
* dot is found and the new position is returned.  Otherwise, if the dot is not
* found, then the old location is returned.
*
* @param[in] img the image containing the dot
* @param[in] max_resolution the maximum resolution of the image
* @param[in] dot the dot to find
* @param[out] new_loc the new location of the dot
* @return true, if the dot is found and <code> new_loc </code> contains the new
* location, false otherwise.
*/

bool TrackDot::find(const Mat& img, const Dot& dot, Point2d& new_loc, double& area) // area: added for checking the number of detected pixels.  may not be necessary later.		
{
	int x, y, y0, yf;
	float radius;
	Mat_<uchar> pixel;
	vector<Point> boundary;
	Point2d& prev_loc = dot.pixel();
	Rect roi = calcRoi(prev_loc, img.size());

	// binarize image
	threshold(img, roi, pixel);

	// avoid boundary problem at y = 0 and y = _rh
	y0 = 1;
	yf = _rh - 1;

	// find (vertically-aligned) boundary pixels
	for(x = 0; x < _rw; ++x) {
		for(y = y0; y < yf; ++y) {
			// does I(x, y) = WHITE && dI(x, y)/dy != 0, where I = intensity
			// note: pixel(row, col) <=> pixel(y, x) <=> I(x, y)
			if(pixel(y, x) && pixel(y - 1, x) - pixel(y + 1, x)) {
				// add location of boundary pixel
				boundary.push_back(Point(x, y));
			}
		}
	}
		
	if(!boundary.empty()) {
		Point2f p;
		cv::minEnclosingCircle(Mat(boundary), p, radius);
		if(radius > _minr && radius < _maxr) {
			// update location only if it passes the size filter
			Point tl;
			Size wholeSize;
			img.locateROI(wholeSize, tl);

			if(wholeSize == img.size()) {
				new_loc = Point2d(roi.x + p.x, roi.y + p.y);
			}
			else {
				new_loc = Point2d(tl.x + p.x, tl.y + p.y);
			}
			
			area = radius; //cv::contourArea(Mat(boundary)); // added for checking the number of detected pixels.  may not be necessary later.
			return true;
		}
	}

	new_loc = prev_loc;
	return false;
}

bool TrackDot::find_pbu(const Mat& img, const Dot& dot, Point2d& new_loc, double& area) // area: added for checking the number of detected pixels.  may not be necessary later.
{
	int x, y, y0, yf, k, thresh, dot_size, ii, cnt=0;
	int hist[256] = { 0 }; //initialize histogram to zero 
	float radius;
	double cx=0, cy=0;
	Mat_<uchar> pixel, dst;
	vector<Point> boundary;
	Point2d& prev_loc = dot.pixel();
	Rect roi = calcRoi(prev_loc, img.size());

	//img(roi).copyTo(dst);
	
		// binarize image
	threshold(img, roi, dst);
	ii=0;

	for(x = 0; x < _rw; ++x) {
		for(y = 0; y < _rh; ++y) {
			if (dst(y,x)>0) {
				cx += x;
				cy += y;
				++ii;
			}
		}
	}

	k=0; 
	
	// added by Ji-Chul
	// It's the case when the tracker couldn't detect the dot.
	// must return false.
	if ( ii == 0 ) {
		new_loc = prev_loc;
		return false;
	}
	

	//std::cout << ii << std::endl;
    radius = sqrt(ii/3.14159);
	Point2f p;
	p.x=cx/double(ii);
	p.y=cy/double(ii);

	Point tl;
	Size wholeSize;
	img.locateROI(wholeSize, tl);

	if(wholeSize == img.size()) {
		new_loc = Point2d(roi.x + p.x, roi.y + p.y);
	}
	else {
		new_loc = Point2d(tl.x + p.x, tl.y + p.y);
	}

	area = radius; //cv::contourArea(Mat(boundary)); // added for checking the number of detected pixels.  may not be necessary later.
	return true;
}

bool TrackDot::find_pbu2(const Mat& img, const Dot& dot, Point2d& new_loc, double& area) // area: added for checking the number of detected pixels.  may not be necessary later.
{
	int x, y, y0, yf, k, thresh, dot_size, ii, cnt=0;
	int hist[256] = { 0 }; //initialize histogram to zero 
	float radius;
	double cx=0, cy=0;
	Mat_<uchar> pixel, dst;
	vector<Point> boundary;
	Point2d& prev_loc = dot.pixel();
	Rect roi = calcRoi(prev_loc, img.size());

	double minVal, maxVal,a,b;
	img(roi).copyTo(dst);
	//minMaxLoc(dst, &minVal, &maxVal); //find minimum and maximum intensities
	/*for(x = 0; x < _rw; ++x) {
		for(y = 0; y < _rh; ++y) {
			++hist(dst(y,x));
		}
	}*/

	/* a=255/(maxVal-minVal);
	ii=0;
	for(x = 0; x < _rw; ++x) {
		for(y = 0; y < _rh; ++y) {
			if ((dst(y,x)-minVal)*a>_thr) {
				dst(y,x)=0;
			}
			else {
				dst(y,x)=WHITE;
				cx += x;
				cy += y;
				++ii;
			}
		}
	}

	k=0; */
	
	// binarize image
		threshold(img, roi, pixel);

	//std::cout << ii << std::endl;
    radius = sqrt(ii/3.14159);
	Point2f p;
	//p.x=cx/double(ii);
	//p.y=cy/double(ii);

	Point tl;
	Size wholeSize;
	img.locateROI(wholeSize, tl);

	if(wholeSize == img.size()) {
		new_loc = Point2d(roi.x + p.x, roi.y + p.y);
	}
	else {
		new_loc = Point2d(tl.x + p.x, tl.y + p.y);
	}

	area = radius; //cv::contourArea(Mat(boundary)); // added for checking the number of detected pixels.  may not be necessary later.
	return true;
}

