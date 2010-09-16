#include <cv.h>
#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlg.h"

#define UPDATE 1
#define INTERKEY 250
#define BAD_LOC -1

using std::string;
using std::stringstream;
using cv::Mat;
using cv::Point2d;
using cv::Scalar;

Tracker::Tracker()
{

}

Tracker::Tracker(TrackingAlg& alg)
{
	algorithm(alg);
}

/** @brief sets the new tracking algorithm */
void Tracker::algorithm(TrackingAlg& alg)
{
	_alg = &alg;
}

bool Tracker::track(Camera& cam, Dots& dots)
{
	int tag;
	bool found_all;
	Mat img;
	ActiveDots::const_iterator dot, stop;
	ActiveDots a = dots.activeDots();

	// track dots in the active set
	found_all = true;
	stop = a.end();
	cam.retrieve(img);
	for(dot = a.begin(); dot < stop; ++dot) {
		tag = (*dot)->tag();
		dots.found(tag) = _alg->find(img, dots[tag], dots.pixel(tag));
		dots.world(tag) = cam.pixelToWorld(dots.pixel(tag));

		if(!dots.found(tag)) {
			found_all = false;
		}
	}

	return found_all;
}

/** @brief initialize dots based on information found in a file */
int Tracker::load(Camera& cam, Dots& dots, const string& file)
{
	return 0;
}

/** @brief initialize dots based on their pixel location */
int Tracker::location(Camera& cam, Dots& dots)
{
	return 0;
}

/** @brief initialize dots based on where the user clicks on screen */
int Tracker::click(Camera& cam, Dots& dots)
{
	int c;
	int tag;
	Mat img;
	string s;
	stringstream ss;
	ActiveDots a = dots.activeDots();
	const string& win = _alg->clickingWindow();
	ClickPoints click_info(cam, dots);

	// set all active dots to not found
	for(size_t i = 0; i < a.size(); ++i) {
		dots.found(a[i]->tag()) = false;
		dots.pixel(a[i]->tag()) = Point2d(BAD_LOC, BAD_LOC);
	}

	// create window and setup mouse callback function
	cv::namedWindow(win);

	// using C version because C++ version does not exist
	cvSetMouseCallback(win.c_str(), click_info.onMouse, &click_info);

	// let user click until 'q' is hit
	s = "";
	tag = 0;
	while(1) {
		c = cv::waitKey(INTERKEY);
		if(c == 'q') {
			// user want to quit
			break;
		}
		else if(c == '\n' || c == '\r') {
			// user wants to place a new dot, get tag info
			ss.str(s);
			ss.seekg(0);
			ss >> tag;
			s = "";

			if(!ss.fail() && dots.isDotActive(tag)) {
				click_info.tag = tag;
				dots.found(tag) = false;
				dots.pixel(tag) = Point2d(BAD_LOC, BAD_LOC);
			}
		}
		else if(c >= 0) {
			// user is still entering tag info
			s += c;
		}

		// show all dots that have been clicked on
		click_info.showScreen(*_alg);
	}

	cvDestroyWindow(win.c_str());

	// count number of dots found
	tag = 0;
	for(size_t i = 0; i < a.size(); ++i) {
		if(a[i]->isFound()) {
			++tag;
		}
	}

	return tag;
}

void Tracker::draw(Camera& cam, Dots& dots, Mat& dst)
{
	Mat src;
	ActiveDots a = dots.activeDots();

	if(cam.retrieve(src)) {
		dst = src.clone();
		for(ActiveDots::const_iterator dot = a.begin(); dot < a.end(); ++dot) {
			_alg->draw(src, *(*dot), dst);
		}
	}
}

const Scalar Tracker::ClickPoints::MOVE_COLOR = Scalar(0, 0, 255);

Tracker::ClickPoints::ClickPoints(Camera& cam, Dots& dots)
{
	tag = dots.activeDots()[0]->tag();
	this->cam = &cam;
	this->dots = &dots;
	mouse_move = Point2d(0, 0);
}

void Tracker::ClickPoints::showScreen(TrackingAlg& alg)
{
	int t;
	Mat img, dst;
	Point2d new_loc;
	ActiveDots a = dots->activeDots();
	const string& win = alg.clickingWindow();

	// take a new undistorted image
	cam->undistort(img);
	if(img.empty()) {
		return;
	}

	// draw all dots that have been clicked on
	dst = img.clone();
	for(size_t i = 0; i < a.size(); ++i) {
		t = a[i]->tag();
		if(dots->pixel(t).x != BAD_LOC && dots->pixel(t).y != BAD_LOC) {
			// keep tracking dot
			dots->found(t) = alg.find(img, (*dots)[t], new_loc);
			if(dots->found(t)) {
				dots->pixel(t) = new_loc;
				dots->world(t) = cam->pixelToWorld(new_loc);
			}

			// draw its updated position
			// ** technically, we should only draw when dot is found
			// but the user won't get any feedback as to whether any
			// of alg's parameters need to be tuned in order to track
			// properly.  Fortunately, we know previous dot location is 
			// not updated (see above), so we can show the user what the 
			// tracking algorithm is trying to do. **
			alg.draw(img, (*dots)[t], dst);
		}
	}

	// draw mouse position and current tag
	stringstream ss;
	ss << tag << " (" << mouse_move.x << "," << mouse_move.y << ")";
	cv::putText(dst, ss.str(), mouse_move, 
		CV_FONT_HERSHEY_PLAIN, 1, MOVE_COLOR);

	cv::imshow(win, dst);
	cv::waitKey(UPDATE);
}

void Tracker::ClickPoints::onMouse(int e, int x, int y, int flags, void* data)
{
	ClickPoints* cp = static_cast<ClickPoints*> (data);
	if(e == CV_EVENT_LBUTTONDOWN) {
		int tag = cp->tag;
		cp->dots->found(tag) = true;
		cp->dots->pixel(tag) = Point2d(x, y);
		cp->dots->world(tag) = cp->cam->pixelToWorld(cp->dots->pixel(tag));
	}
	else if(e == CV_EVENT_MOUSEMOVE) {
		cp->mouse_move = Point2d(x, y);
	}
}