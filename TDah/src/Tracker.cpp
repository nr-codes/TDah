#include <fstream>
#include "Dots.h"
#include "Camera.h"
#include "Tracker.h"
#include "TrackingAlg.h"

#define UPDATE 1
#define INTERKEY 1000
#define BAD_LOC -1

using std::ifstream;
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

	found_all = true;
	if(!cam.retrieve(img)) {
		return false;
	}

	// track dots in the active set
	ActiveDots& a = dots.activeDots();
	for(dot = a.begin(), stop = a.end(); dot < stop; ++dot) {
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
	int i;
	stringstream ss;
	string line, sx, sy, sz;
	double x, y, z;
	ifstream f;

	f.open(file.c_str());
	if(!f.is_open())
		return false;

	i = 0;
	while(!f.eof()) {
		std::getline(f, line);
		size_t pos1 = line.find(' ');
		size_t pos2 = line.find(' ',  pos1 + 1);

		if(pos1 == string::npos)
			return false;
		if(pos2 == string::npos)
			pos2 = line.size();

		sx = line.substr(0, pos1);
		sy = line.substr(pos1 + 1, pos2 - pos1 - 1);
		if(pos2 == line.size())
			sz == "";
		else
			sz = line.substr(pos2 + 1, line.size() - pos2 - 1);

		// read in x value
		ss.str(sx);
		ss.seekg(0);
		ss >> x;
		if(ss.fail())
			return false;

		// read in y value
		ss.str(sy);
		ss.seekg(0);
		ss >> y;
		if(ss.fail())
			return false;

		// read in z value
		ss.str(sz);
		ss.seekg(0);
		ss >> z;

		// set the corresponding dot's location
		//world.push_back(Point3f(x, y , 0));
		++i;
	}

	f.close();
	return i;
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
	const string& win = _alg->clickingWindow();
	ClickPoints click_info(cam, dots);

	// set all active dots to not found
	ActiveDots& a = dots.activeDots();
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

	if(cam.retrieve(src)) {
		// copy img into dst
		if(src.type() == CV_8UC1) {
			cv::cvtColor(src, dst, CV_GRAY2BGR);
		}
		else {
			dst = src.clone();
		}

		ActiveDots& a = dots.activeDots();
		for(ActiveDots::const_iterator dot = a.begin(); dot < a.end(); ++dot) {
			_alg->draw(src, *(*dot), dst);
		}
	}
}

string Tracker::str(Dots& dots)
{
	static stringstream ss;

	ss.str("");
	ActiveDots& a = dots.activeDots();
	for(size_t i = 0; i < a.size(); ++i) {
		ss << a[i]->tag() << " " << a[i]->imageNbr() << " " << a[i]->isFound() <<
			" " << a[i]->pixelX() << " " << a[i]->pixelY() << 
			" " << a[i]->worldX() << " " << a[i]->worldY() <<  
			" " << a[i]->worldZ() << " " << a[i]->timeStamp()
			<< std::endl;
	}

	return ss.str();
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
	const string& win = alg.clickingWindow();

	// take a new undistorted image
	cam->undistort(img);
	if(img.empty()) {
		return;
	}

	// copy img into dst
	if(img.type() == CV_8UC1) {
		cv::cvtColor(img, dst, CV_GRAY2BGR);
	}
	else {
		dst = img.clone();
	}

	// draw all dots that have been clicked on
	ActiveDots& a = dots->activeDots();
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