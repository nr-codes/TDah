#include <iomanip>
#include "Calibration.h"

using std::stringstream;
using std::vector;
using std::string;
using cv::Mat;
using cv::Point2f;
using cv::Point;
using cv::Size;
using cv::TermCriteria;
using cv::VideoCapture;
using cv::Range;
using cv::Mat_;
using cv::Vec4i;
using cv::Scalar;

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

static string main_win;
static const string param_win = "Polk Dots Parameters";
static const Scalar color1( rand()&255, rand()&255, rand()&255 );
static const Scalar color2( rand()&255, rand()&255, rand()&255 );
static const Scalar color3( rand()&255, rand()&255, rand()&255 );

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
	cv::createTrackbar("Canny threshold 1", param_win, thr1, WHITE);
	cv::createTrackbar("Canny threshold 2", param_win, thr2, WHITE);
}

/**
* Sorts the points using a specified origin and the L1 norm
*/

static void distanceSort(const vector<Point2f>& points, const Point2f& origin, 
							   vector<int>& index)
{
	Point2f d;
	vector<double> distances;
	for(size_t j = 0; j < points.size(); j++) {
		d = points[j] - origin;
		distances.push_back(std::abs(d.x) + std::abs(d.y));
	}

	cv::sortIdx(Mat(distances), Mat(index), CV_SORT_EVERY_COLUMN);
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

static void draw_points(const vector<int>& indices, 
						const vector<Point2f>& positions, 
						const vector<Point2f>& pixels,
						Mat& dst, 
						Point2f& shift_x, Point2f& shift_y, Point2f& shift_i)
{
	vector<int> labels;
	stringstream ss;
	CV_Assert(indices.size() == pixels.size() && 
		pixels.size() == positions.size());

	labels.assign(indices.size(), 0);
	for(size_t i = 0; i < labels.size(); ++i) {
		labels[indices[i]] = i;
	}

	ss << std::setprecision(3);
	for(size_t i = 0; i < pixels.size(); ++i) {
		ss.str("");
		ss << labels[i];
		cv::putText(dst, ss.str(), pixels[i], 
			cv::FONT_HERSHEY_SIMPLEX, 0.30, Scalar(0, 0, 255), 1, CV_AA);

		ss.str("");
		ss << positions[i].x;
		cv::putText(dst, ss.str(), pixels[i] + shift_x, 
			cv::FONT_HERSHEY_SIMPLEX, 0.25, color1, 1, CV_AA);

		ss.str("");
		ss << positions[i].y;
		cv::putText(dst, ss.str(), pixels[i] + shift_y, 
			cv::FONT_HERSHEY_SIMPLEX, 0.25, color2, 1, CV_AA);

		ss.str("");
		ss << i;
		cv::putText(dst, ss.str(), pixels[i] + shift_i, 
			cv::FONT_HERSHEY_SIMPLEX, 0.25, color3, 1, CV_AA);
	}

	cv::line(dst, pixels[indices[0]], pixels[indices[1]], color2, 1, CV_AA);
	cv::line(dst, pixels[indices[0]], pixels[indices[2]], color1, 1, CV_AA);
}

static void draw_coords(Mat& src, vector<Point2f>& centers, Mat& dst, 
						const Size& grid)
{
	Mat sortIdx;
	vector<double> indx;

	double w = std::abs(src.at<Point2f>(0, 0).x) + 1;
	for(size_t i = 0; i < centers.size(); ++i) {
		double x = std::abs(src.at<Point2f>(i, 0).x);
		double y = std::abs(src.at<Point2f>(i, 0).y);
		indx.push_back(x + y * w);//grid.width);
	}
	cv::sortIdx(Mat(indx), sortIdx, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING );

	for(size_t i = 0; i < indx.size(); ++i) {
		indx[i] = sortIdx.at<int>(i, 0);
	}

	stringstream *ss;
	int k = 0;
	for(int i = 0; i < src.rows; ++i) {
		for(int j = 0; j < src.cols; ++j) {
			float x = src.at<Point2f>(i, j).x;
			float y = src.at<Point2f>(i, j).y;
			int m = sortIdx.at<int>(k, 0);

			ss = new stringstream;
			*ss << k;
			cv::putText(dst, ss->str(), centers[m], 
				cv::FONT_HERSHEY_SIMPLEX, 0.30, color3, 1, CV_AA);
			k++;
			delete ss;

			ss = new stringstream;
			*ss << std::setprecision(2) << x;
			cv::putText(dst, ss->str(), centers[m] + Point2f(-10, -10), 
				cv::FONT_HERSHEY_SIMPLEX, 0.20, color3, 1, CV_AA);
			delete ss;

			ss = new stringstream;
			*ss << std::setprecision(2) << y;
			cv::putText(dst, ss->str(), centers[m] + Point2f(10, 10), 
				cv::FONT_HERSHEY_SIMPLEX, 0.20, color3, 1, CV_AA);
			delete ss;

			ss = new stringstream;
			*ss << m;
			cv::putText(dst, ss->str(), centers[m] + Point2f(10, 0), 
				cv::FONT_HERSHEY_SIMPLEX, 0.20, color2, 1, CV_AA);
			delete ss;
		}
	}
}

int Calibration::getPolkaDotViews(VideoCapture* cam, string title)
{
	// NOTE DOES NOT WORK
	CV_Assert(false);

	find_chessboard.grid = Size(4, 3); // TODO DELETE

	Mat bgr, edges;
	vector<vector<Point>> contours;
	vector<float> radii;
	vector<Point2f> centers;
	vector<Vec4i> hierarchy;
	int ndots, input, n;
	bool prompt;

	// initialize values
	main_win = "Calibrating: " + title;
	n = views.n;
	prompt = views.prompt;
	input = 0;
	ndots = find_chessboard.grid.area();
	radii.assign(ndots, 0);
	centers.assign(ndots, Point2f());
	polka_dots.dilate = DILATE;
	polka_dots.erode = ERODE;
	polka_dots.thr1 = THR1;
	polka_dots.thr2 = THR2;
	create_ui(*this);

	while(1) {
		*cam >> bgr;
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

			// find centers and radii
			int child = hierarchy[i][CHILD];
			for(int c = child, r = 0; c > -1; c = hierarchy[c][0], r++) {
				cv::minEnclosingCircle(Mat(contours[c]), centers[r], radii[r]);
				cv::drawContours(bgr, contours, c, color1, 2);
			}

			// is it really the polka dot grid
			Scalar mean, stddev;
			cv::meanStdDev(Mat(radii), mean, stddev);
			if(mean[0] < MIN_RAD && stddev[0] > STD_ERR) {
				continue;
			}
			// should also do a check for distance between two points

			// everything appears to check out, save view
			if(views.pixel.empty())
				views.pixel.push_back(centers);
			else
				views.pixel[0] = centers;

			vector<int> sorted_indices(ndots, 0);

			// sort the points according to distance using any reference point
			size_t ref = 0;
			Mat(centers) -= Scalar(centers[ref].x, centers[ref].y);
			distanceSort(centers, Point2f(0, 0), sorted_indices);
			
			// now sort using a corner point
			ref = sorted_indices.back();
			Mat(centers) -= Scalar(centers[ref].x, centers[ref].y);
			distanceSort(centers, Point2f(0, 0), sorted_indices);

			// rotate image frame to polka dot frame
			double scale;
			double angle;
			Point2f dir1;
			Point2f dir2;
			Range col1(0, 1);
			Range col2(1, 2);
			Range rows12 = Range::all();
			Mat_<double> rot =  Mat_<double>::zeros(2, 2);

			dir1 = centers[sorted_indices[1]];
			dir2 = centers[sorted_indices[2]];
			scale = 0.5*(std::sqrt(dir1.x*dir1.x + dir1.y*dir1.y) +
				std::sqrt(dir2.x*dir2.x + dir2.y*dir2.y));
			angle = atan2(dir1.x, dir1.y);

			rot(0, 0) = cos(angle);
			rot(1, 0) = -sin(angle);
			rot(0, 1) = sin(angle);
			rot(1, 1) = cos(angle);


			cv::normalize(rot(rows12, col1), rot(rows12, col1));
			cv::normalize(rot(rows12, col2), rot(rows12, col2));
			cv::transform(Mat(centers), Mat(centers), rot);

			std::cout << "Rot"  << std::endl;
			for(int i = 0; i < rot.rows; i++) {
				for(int j = 0; j < rot.cols; j++) {
					//std::cout << rot(i, j) << "\t";
				}
				//std::cout << std::endl;
			}
			std::cout << std::endl;

			Mat(centers) /= scale;
			double lastx = std::abs(centers[sorted_indices.back()].x);
			double lasty = std::abs(centers[sorted_indices.back()].y);
			double expectedx = find_chessboard.grid.width - 1;
			double expectedy = find_chessboard.grid.height - 1;

			std::cout << "expected: " << expectedx << "\t" << expectedy << std::endl;
			std::cout << "x: " << 
				(std::abs(centers[sorted_indices.back()].x)) << "\t"
				<< 	std::abs(lastx - expectedx)
				<< std::endl;

			std::cout << "y: " << 
				(std::abs(centers[sorted_indices.back()].y)) << "\t"
				<< std::abs(lasty - expectedy)
				<< std::endl;

			// show image
			vector<double> order1;
			for(size_t k = 0; k < centers.size(); ++k) {
				order1.push_back(std::abs(centers[k].x) + std::abs(centers[k].y) * 
					(lastx+1));
			}

			cv::sortIdx(Mat(order1), Mat(sorted_indices), 
				CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING );
			Mat temp1 = bgr.clone();
			draw_points(sorted_indices, centers, views.pixel.back(), temp1,
				Point2f(0, -10), Point2f(0, 10), Point2f(15, 0));
			std::cout << "before" << std::endl;
			cv::imshow(main_win, temp1);

			cv::waitKey(0);

			if(std::abs(lastx - expectedx) > 0.5 &&
				std::abs(lasty - expectedy) > 0.5 ) {
					// rotate if not in grid layout
					rot(0, 0) = cos(CV_PI / 2);
					rot(1, 0) = -sin(CV_PI / 2);
					rot(0, 1) = sin(CV_PI / 2);
					rot(1, 1) = cos(CV_PI / 2);
					cv::transform(Mat(centers), Mat(centers), rot);
					std::cout << "rotate by 90" << std::endl;
			}

			distanceSort(centers, Point2f(0, 0), sorted_indices);

			vector<double> order;
			for(size_t k = 0; k < centers.size(); ++k) {
				order.push_back(std::abs(centers[k].x) + std::abs(centers[k].y) * 
					find_chessboard.grid.width);
			}

			cv::sortIdx(Mat(order), Mat(sorted_indices), 
				CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING );

			// show image
			Mat temp = bgr.clone();
			draw_points(sorted_indices, centers, views.pixel.back(), temp,
				Point2f(0, -10), Point2f(0, 10), Point2f(15, 0));
			cv::imshow(main_win, temp);

			std::cout << "afters" << std::endl;
			input = cv::waitKey(0);
			if(input == 'q') break;



/*

			// the farthest dot is always a corner dot in the grid pattern
			Point2f corner1 = centers[sorted_indices.back()];

			// sort the points from closest to farthest using a corner point
			distanceSort(centers, corner1, sorted_indices);

			// the two closest points (not including the origin) will tell us
			// the direction & orientation of the pattern's reference frame
			double dist;
			Range col1(0, 1);
			Range col2(1, 2);
			Range rows12 = Range::all();
			Mat_<double> rot =  Mat_<double>::zeros(2, 2);

			Point2f dir1 = centers[sorted_indices[1]] - corner1;
			Point2f dir2 = centers[sorted_indices[2]] - corner1;

			rot(0, 0) = dir1.x;
			rot(1, 0) = dir1.y;
			rot(0, 1) = dir2.x;
			rot(1, 1) = dir2.y;

			dist = 0.5 * (cv::norm(rot(rows12, col1)) + 
				cv::norm(rot(rows12, col2)));

			cv::normalize(rot(rows12, col1), rot(rows12, col1));
			cv::normalize(rot(rows12, col2), rot(rows12, col2));

			Mat dst;
			cv::transform(Mat(centers) - Scalar(corner1.x, corner1.y), dst, rot);
			dst *= 1/dist;


			if(std::abs(dst.at<Point2f>(ndots - 1, 0).x) 
				!= find_chessboard.grid.width - 1 &&
				std::abs(dst.at<Point2f>(ndots - 1, 0).y) 
				!= find_chessboard.grid.height - 1) {
					rot(0, 0) = cos(CV_PI / 2);
					rot(1, 0) = sin(CV_PI / 2);
					rot(0, 1) = -sin(CV_PI / 2);
					rot(1, 1) = cos(CV_PI / 2);
					//cv::transform(dst, dst, rot);
			}

  			draw_coords(dst, centers, bgr, find_chessboard.grid);

			cv::drawContours( bgr, contours, i, color2, 1, 8, hierarchy, 0 );
			cv::imshow(main_win, bgr);

			input = cv::waitKey(10);
			if(input == 'q') break;
			cv::waitKey(0);
*/
		}

		if(input == 'q' || cv::waitKey(10) == 'q') break;

		//cv::imshow(main_win, bgr);
		contours.clear();
	}

	return 0;
}