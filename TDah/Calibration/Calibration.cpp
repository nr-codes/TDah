/** @file Calibration.cpp
*/

#include <fstream>
#include "Calibration.h"

using std::vector;
using std::string;
using std::ifstream;
using std::stringstream;
using cv::Mat;
using cv::Point2f;
using cv::Point3f;
using cv::Size;
using cv::TermCriteria;
using cv::VideoCapture;
using cv::Range;
using cv::Mat_;

/** @brief the constructor (calls setDefaults) */
Calibration::Calibration()
{
	setDefaults();
}

/** @brief the constructor (calls setDefaults for all other parameters) */
Calibration::Calibration(Size grid, bool save_views)
{
	// must call this first to avoid resetting the desired values
	setDefaults();

	// set the desired values
	find_chessboard.grid = grid;
	views.save_views = save_views;
}

/**
* This functions will set the following members to these default values:
*
*	views.save_views = false;
*
*	find_chessboard.flags = 0;
*	find_chessboard.grid = Size(0, 0);
*
*	sub_pixel.win = WIN_SIZE;
*	sub_pixel.zz = ZERO_ZNE;
*	sub_pixel.crit = ERR_TOL;
*
*	calib_cam.flags = 0;
*
*	set_pnp.useExtGuess = false;
*
* These parameters are passed to various functions to refine the
* calibration, see the OpenCV documentation for more details as to
* how each function uses these parameters.
*/

void Calibration::setDefaults()
{
	views.save_views = false;

	find_chessboard.flags = 0;
	find_chessboard.grid = Size(0, 0);

	sub_pixel.win = WIN_SIZE;
	sub_pixel.zz = ZERO_ZNE;
	sub_pixel.crit = ERR_TOL;

	calib_cam.flags = 0;

	solve_pnp.useExtGuess = false;

	polka_dots.dilate = 0;
	polka_dots.erode = 0;
	polka_dots.thr1 = 0;
	polka_dots.thr2 = 0;
}

void Calibration::setGridSize(Size grid)
{
	find_chessboard.grid = grid;
}

void Calibration::saveViews(bool save_views)
{
	views.save_views = save_views;
}

void Calibration::clearVectors()
{
	views.chessboards.clear();
	calib_cam.rvecs.clear();
	calib_cam.tvecs.clear();
}

bool Calibration::getWorldPoints(string filename, vector<Point3f>& world)
{
	stringstream ss;
	string line, sx, sy;
	float x, y;
	ifstream file;

	file.open(filename.c_str());
	if(!file.is_open()) return false;

	while(!file.eof()) {
		std::getline(file, line);
		size_t pos = line.find(' ');
		if(pos == string::npos) return false;

		sx = line.substr(0, pos);
		sy = line.substr(pos + 1, line.size() - pos - 1);

		// read in x value
		ss.str(sx);
		ss.seekg(0);
		ss >> x;
		if(ss.fail()) return false;

		// read in y value
		ss.str(sy);
		ss.seekg(0);
		ss >> y;
		if(ss.fail()) return false;

		world.push_back(Point3f(x, y , 0));
	}

	file.close();
	return true;
}

/** TODO update doc AND replace with cv equivalents which do doubles
* A calibration routine for determing the intrinsic parameters of an image
* based on a chessboard grid pattern.  The routine will determine the 
* parameters from the <code> n </code> images taken by the camera.
*
* @param[in] calib the calibration parameters with find_chessboard.grid set
* @param[in] n the number of image to take
*
* @note if calib.find_chessboard.grid is not correctly set, then the results 
* will not mean anything.  It is very important to set this parameter to
* the correct number of rows and columns in the chessboard pattern.
*
* @note optionally, the calib.views.save_views parameter can be set, if
* the chessboard images corresponding to the world and pixel points want to
* be viewed
*
* @note calib.calib_cam.rvecs and calib.calib_cam.tvecs will be filled with
* world to camera frame rotation and translation vectors.  The rotation matrix
* can be obtained from the rotation vector using the OpenCV Rodrigues function.
*/

double Calibration::getIntrinsics(Size img_size)
{
	// get various parameters
	int flags = calib_cam.flags;
	Calibration::VVP3f& w = views.world;
	Calibration::VVP2f& p = views.pixel;
	vector<Mat>& rvecs = calib_cam.rvecs;
	vector<Mat>& tvecs = calib_cam.tvecs;

	Mat& A = intrinsic_params.A;
	Mat& k = intrinsic_params.k;
	
	// calibrate the camera
	return cv::calibrateCamera(w, p, img_size, A, k, rvecs, tvecs, flags);
}

/** TODO update doc AND replace with cv equivalents which do doubles
* A calibration routine for determing the extrinsic parameters of an image
* based on a chessboard grid pattern.  By default the world frame is set by
* the getViews member function of Calibration.  By supplying a
* 3x4 transformation matrix [R|t], the frame can be rotated and shifted to 
* a new location.  The transformation matrix must take points and vectors
* in the desired frame to the default frame used by getViews.  This is done
* to be consistent with OpenCV's definition of it's transformation as being
* from a world frame to the camera frame.
*
* @param[in] calib the calibration parameters with find_chessboard.grid set
* @param[in] Tr the transformation matrix from new frame to getViews frame
*
* @note if calib.find_chessboard.grid is not correctly set, then the results 
* will not mean anything.  It is very important to set this parameter to
* the correct number of rows and columns in the chessboard pattern.
*
* @note optionally, the calib.views.save_views parameter can be set, if
* the chessboard images corresponding to the world and pixel points want to
* be viewed
*
* @note by default the Tr is empty and not used
*/

double Calibration::getExtrinsics(const Mat& Tr)
{
	// set/create various parameters
	Mat rvec;
	Mat w(views.world.front());
	Mat p(views.pixel.front());

	Mat& A = intrinsic_params.A;
	Mat& k = intrinsic_params.k;
	Mat& R = extrinsic_params.R;
	Mat& t = extrinsic_params.t;

	// get extrinsic parameters
	cv::solvePnP(w, p, A, k, rvec, t, solve_pnp.useExtGuess);
	cv::Rodrigues(rvec, R);

	// convert default coordinate system to new one
	if(!Tr.empty()) {
		CV_Assert(Tr.rows == 3 && Tr.cols == 4);
		Mat T_Tr = Mat(Tr, Range(0, 3), Range(3, 4));
		Mat R_Tr = Mat(Tr, Range(0, 3), Range(0, 3));

		t += R*T_Tr;
		R *= R_Tr;
	}

	return 0.;
}
