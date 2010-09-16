#include <iomanip>
#include "Calibration.h"

#define ERODE 0
#define DILATE 0
#define THR1 18 /**< default thresohld parameters */
#define THR2 96 /**< default threshold parameter */
#define MAX_ITER 5
#define WHITE 255
#define APERTURE 3 /**< default aperture setting */
#define CHILD 2
#define NEXT 0
#define STD_ERR 5
#define MIN_RAD 3
#include <iostream>
#include <cmath>

static const std::string main_win = "Searching for a Grid...";
static const std::string param_win = "Grid Parameters";
static const cv::Scalar RED( 0, 0, 255 );
static const cv::Scalar GREEN( 0, 255, 0 );

static struct {
	std::vector<cv::Rect> rects;
	std::vector<bool> present;
	std::vector<int> order;
	cv::Mat img;
} vw;

using std::stringstream;
using std::vector;
using std::string;
using cv::Mat;
using cv::Point2f;
using cv::Point;
using cv::Size;
using cv::Rect;
using cv::TermCriteria;
using cv::VideoCapture;
using cv::Range;
using cv::Mat_;
using cv::Vec4i;
using cv::Scalar;

static Point2f rect_center(int i)
{
	float x = static_cast<float> (vw.rects[i].x);
	float y = static_cast<float> (vw.rects[i].y);
	float w = static_cast<float> (vw.rects[i].width / 2.);
	float h = static_cast<float> (vw.rects[i].height / 2.);

	return Point2f(x + w, y + h);
}

static int find_rect(Point& point, vector<Rect>& rects)
{
	for(size_t i = 0; i < rects.size(); ++i) {
		if(rects[i].contains(point)) {
			return i;
		}
	}

	return -1;
}

static void show_rects()
{
	stringstream ss;
	Mat dst = vw.img.clone();
	vector<int> reverse_lookup(vw.rects.size(), -1);

	// get user click order for each clicked rectangle
	for(size_t i = 0; i < vw.order.size(); ++i) {
		reverse_lookup[vw.order[i]] = i + 1;
	}

	// draw green, if clicked, and red otherwise
	for(size_t i = 0; i < vw.rects.size(); ++i) {
		if(reverse_lookup[i] > -1) {
			cv::rectangle(dst, vw.rects[i].tl(), 
				vw.rects[i].br(), GREEN);

			ss.str("");
			ss << reverse_lookup[i];
			cv::putText(dst, ss.str(), vw.rects[i].tl(), 
				cv::FONT_HERSHEY_PLAIN, 1, GREEN, 1, CV_AA);

			cv::circle(dst, rect_center(i), 2, GREEN, CV_FILLED, CV_AA);
		}
		else {
			cv::rectangle(dst, vw.rects[i].tl(), 
				vw.rects[i].br(), RED);
		}
	}

	cv::imshow(main_win, dst);
	cv::waitKey(1);
}

void mouse_click(int e, int x, int y, int flags, void* param)
{
	stringstream ss;
	if(vw.rects.empty()) {
		return;
	}
	else if(e == CV_EVENT_LBUTTONDOWN) {
		int i = find_rect(Point(x, y), vw.rects);
		if(i < 0 || vw.present[i]) {
			return;
		}

		vw.order.push_back(i);
		vw.present[i] = true;
		show_rects();
	}
	else if(e == CV_EVENT_RBUTTONDOWN) {
		int i = find_rect(Point(x, y), vw.rects);
		if(i < 0 || !vw.present[i]) {
			return;
		}

		// search for place in grid
		size_t j;
		for(j = 0; i != vw.order[j]; ++j);

		// remove and update drawing
		for(size_t k = j; k < vw.order.size(); ++k) {
			i = vw.order[k];
			vw.present[i] = false;
		}
		vw.order.erase(vw.order.begin() + j, vw.order.end());
		show_rects();
	}
}

/**
* Creates a simple window that allows the user to change the number of 
* dilations and erosions to do and set the two Canny threshold levels.
*/
static void create_ui( Calibration& calib )
{
	int* dilate = &calib.polka_dots.dilate;
	int* erode = &calib.polka_dots.erode;
	int* thr1 = &calib.polka_dots.thr1;
	int* thr2 = &calib.polka_dots.thr2;

	cv::namedWindow(main_win, 0); // allow resizing of image
	cv::namedWindow(param_win);
	cv::createTrackbar("dilate", param_win, dilate, MAX_ITER);
	cv::createTrackbar("erode", param_win, erode, MAX_ITER);
	cv::createTrackbar("threshold 1", param_win, thr1, WHITE);
	cv::createTrackbar("threshold 2", param_win, thr2, WHITE);
}

static void process_image(const Calibration& calib, const Mat& src, Mat& edges)
{
	Mat gr;
	int dilate = calib.polka_dots.dilate;
	int erode = calib.polka_dots.erode;
	int thr1 = calib.polka_dots.thr1;
	int thr2 = calib.polka_dots.thr2;

	// image processing
	cv::cvtColor(src, gr, CV_BGR2GRAY);
	cv::pyrDown(gr, edges);
	cv::pyrUp(edges, gr);
	cv::Canny(gr, edges, thr1, thr2, APERTURE, true);
	cv::dilate(edges, gr, Mat(), Point(-1, -1), dilate);
	cv::erode(gr, edges, Mat(), Point(-1, -1), erode);

	cv::imshow("edges", edges);
}

static bool is_grid(int ndots, const vector<Vec4i>& hier, int i)
{
	int c, n;

	for(n = 0, c = hier[i][CHILD]; c > -1; ++n, c = hier[c][NEXT]);
	if(ndots != n)
		return false;

	return true;
}

int Calibration::getClickViews(VideoCapture& cam)
{
	Mat bgr, edges;
	vector<vector<Point>> contours;
	vector<Rect> rects;
	vector<Vec4i> hierarchy;
	int ndots, input;

	// initialize values
	input = 0;
	ndots = find_chessboard.grid.area();
	polka_dots.dilate = DILATE;
	polka_dots.erode = ERODE;
	polka_dots.thr1 = THR1;
	polka_dots.thr2 = THR2;
	create_ui(*this);

	while(1) {
		cam >> bgr;
		if(bgr.empty()) {
			continue;
		}

		process_image(*this, bgr, edges);

		// find contours
		cv::findContours(edges, contours, hierarchy, 
			CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

		for(size_t i = 0; i < hierarchy.size(); ++i) {
			if(!is_grid(ndots, hierarchy, i)) {
				continue;
			}

			// found a grid pattern
			vw.rects.clear();
			vw.present.assign(ndots, false);
			vw.order.clear();
			vw.img = bgr;

			int child = hierarchy[i][CHILD];
			for(int c = child, r = 0; c > -1; c = hierarchy[c][0], r++) {
				vw.rects.push_back(cv::boundingRect(Mat(contours[c])));
			}
			show_rects();

			// turn on mouse clicks until user presses a key and then turn off
			cvSetMouseCallback(main_win.c_str(), mouse_click);
			input = cv::waitKey(0);
			cvSetMouseCallback(main_win.c_str(), NULL);

			if(vw.order.size() == ndots) {
				// organize points in view according to user click sequence
				vector<Point2f> centers;
				for(vector<int>::const_iterator it = vw.order.begin(); 
					it < vw.order.end(); ++it) {
						centers.push_back(rect_center(*it));
				}
				views.pixel.push_back(centers);

				// user has ordered dots, go to next view
				break;
			}
		}

		if(input == 'q' || cv::waitKey(50) == 'q') break;

		cv::imshow(main_win, bgr);
		cv::waitKey(1);

		contours.clear();
	}

	return views.pixel.size();
}