/** @file Calibration.cpp
*/

#include <iostream>
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
using cv::FileStorage;
using cv::Scalar;

/** @brief the constructor (calls setDefaults) */
Calibration::Calibration()
{
	setDefaults();
}

/** @brief the constructor (calls setDefaults for all other parameters) */
Calibration::Calibration(Size grid, int nimgs, bool prompt, bool save_views)
{
	// must call this first to avoid resetting the desired values
	setDefaults();

	// set the desired values
	find_chessboard.grid = grid;
	views.n = nimgs;
	views.prompt = prompt;
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
	views.n = 0;
	views.prompt = false;
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

	intrinsic_params.file = "";
	extrinsic_params.file = "";
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
	views.imgs.clear();
	calib_cam.rvecs.clear();
	calib_cam.tvecs.clear();
}

/**
* The default world coordinate system is used where the distance between two
* neighboring corners defines a unit of change in the x and y directions.
* Note that if the grid pattern is not square, then there will be two different
* units, one in the x and another in the y.  As further illustrated in the
* ascii-art below, the x-axis runs positive from left-to-right across columns 
* of an image and the y-axis is positive from bottom-to-top along the rows 
* of an image.  This is commonly the xy-frame that is traditionally used in 
* the math and sciences.  The origin of this frame is always located at 
* the first pixel location found by the OpenCV 
* <code> findChessboardCorners </code> function.
*
*		  (0,0) o-------o (1, 0)
*				|		|
*				|		|
*				|		|
*		 (0,-1) o-------o (1,-1)
*
*	^
*	| y
*	o---> x
*
*/

void Calibration::xyWorldPoints()
{
	int cols = find_chessboard.grid.width;
	int npts = find_chessboard.grid.area();
	vector<Point3f> world_loc;
    for(int i = 0; i < npts; ++i) {
        float x = (float) (i % cols);
        float y = (float) (-i / cols);
        world_loc.push_back(Point3f(x, y, 0));
    }

	views.world.assign(views.n, world_loc);
}

bool Calibration::loadWorldPoints(string filename)
{
	vector<Point3f> world;
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

	views.world.assign(views.n, world);
	return world.size() == find_chessboard.grid.area();
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

void Calibration::reproject(Scalar pixel_color, Scalar reproj_color)
{
	stringstream ss;
	size_t nimgs = views.imgs.size();
	size_t npxs = views.pixel.size();
	size_t nwls = views.world.size();

	if(nimgs == npxs && nimgs == nwls) {
		// setup parameters
		Mat r;
		cv::Rodrigues(extrinsic_params.R, r);
		Mat& t = extrinsic_params.t;
		Mat& A = intrinsic_params.A;
		Mat& k = intrinsic_params.k;
		vector<Point2f> pixel;

		// iterate through each image
		for(size_t i = 0; i < nimgs; ++i) {
			// project from world to image frame
			pixel.clear();
			cv::projectPoints(Mat(views.world[i]), r, t, A, k, pixel);

			// get original world and pixel values from image views
			vector<Point3f>& w = views.world[i];
			vector<Point2f>& p = views.pixel[i];

			// get the image number in string format
			ss.str("");
			ss.seekp(0);
			ss.seekg(0);
			ss << "Image: " << (i + 1);
			std::cout << ss.str() << std::endl;

			for(size_t j = 0; j < views.pixel[i].size(); ++j) {
				// iterate over each point
				cv::circle(views.imgs[i], p[j], 5, pixel_color);
				cv::circle(views.imgs[i], pixel[j], 3, 
					reproj_color, CV_FILLED);

				std::cout << "  px=" << p[j].x << " rx=" << pixel[j].x
					<< " ex=" << p[j].x - pixel[j].x 
					<< " py=" << p[j].y << " ry=" << pixel[j].y
					<< " ey=" << p[j].y - pixel[j].y << std::endl;
			}

			Mat diff;
			cv::absdiff(Mat(p), Mat(pixel), diff);

			Scalar mean, stddev;
			cv::meanStdDev(diff, mean, stddev);
			std::cout << "average error |(px,py) - (rx,ry)|=(" << mean[0] << 
				","  << mean[1] << ")" << " +/- (" << stddev[0] << 
				","  << stddev[1] << ")" << std::endl;

			cv::imshow(ss.str(), views.imgs[i]);
			cv::waitKey(1);
		}
	}
}

bool Calibration::writeIntrinsics()
{
	return writeIntrinsics(intrinsic_params.file, 
		intrinsic_params.A, intrinsic_params.k);
}

bool Calibration::readIntrinsics()
{
	return readIntrinsics(intrinsic_params.file, 
		intrinsic_params.A, intrinsic_params.k);
}

bool Calibration::writeIntrinsics(const string& file, const Mat& A, 
								  const Mat& k)
{
	if(file.empty() || A.empty() || k.empty()) {
		// info is missing
		return false;
	}

	FileStorage fs(file, FileStorage::WRITE);
	if(fs.isOpened()) {
		CvMat _A = A;
		CvMat _k = k;

		// write the matrices with the given names
		fs.writeObj("Camera Matrix", &_A);
		fs.writeObj("Distortion Coefficients", &_k);
		return true;
	}

	return false;
}

bool Calibration::readIntrinsics(const string& file, Mat& A, Mat& k)
{
	CvMat* _A = NULL;
	CvMat* _k = NULL;

	if(file.empty()) {
		return false;
	}

	FileStorage fs(file, FileStorage::READ);

	if(fs.isOpened()) {
		_A = static_cast<CvMat*> (
			fs["Camera Matrix"].readObj());
		_k = static_cast<CvMat*> 
			(fs["Distortion Coefficients"].readObj());
	}

	A = _A == NULL ? Mat() : Mat(_A);
	k = _k == NULL ? Mat() : Mat(_k);

	return !A.empty() && !k.empty();
}

bool Calibration::writeExtrinsics(const string& file, const Mat& R, 
								  const Mat& t)
{
	if(file.empty() || R.empty() || t.empty()) {
		// info is missing
		return false;
	}

	FileStorage fs(file, FileStorage::WRITE);
	if(fs.isOpened()) {
		CvMat _R = R;
		CvMat _t = t;

		// write the matrices with the given names
		fs.writeObj("Rotation Matrix", &_R);
		fs.writeObj("Translation Vector", &_t);
		return true;
	}

	return false;
}

bool Calibration::writeExtrinsics()
{
	return writeIntrinsics(extrinsic_params.file, 
		extrinsic_params.R, extrinsic_params.t);
}

bool Calibration::readExtrinsics()
{
	return readIntrinsics(extrinsic_params.file, 
		extrinsic_params.R, extrinsic_params.t);
}

bool Calibration::readExtrinsics(const string& file, Mat& R, Mat& t)
{
	CvMat* _R = NULL;
	CvMat* _t = NULL;

	if(file.empty()) {
		return false;
	}

	FileStorage fs(file, FileStorage::READ);
	if(fs.isOpened()) {
		_R = static_cast<CvMat*> (
			fs["Rotation Matrix"].readObj());
		_t = static_cast<CvMat*> 
			(fs["Translation Vector"].readObj());
	}

	R = _R == NULL ? Mat() : Mat(_R);
	t = _t == NULL ? Mat() : Mat(_t);

	return !R.empty() && !t.empty();
}