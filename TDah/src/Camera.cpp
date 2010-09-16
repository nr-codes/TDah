#include "Camera.h"
#include "Dots.h"

using cv::Mat;
using cv::Mat_;
using cv::Point2d;
using cv::Point3d;
using cv::VideoCapture;

/**
* The constructor initializes the camera's intrinsic and
* extrinsic parameters so that the world and image frame
* are aligned, that is, the rotation matrix (R) and camera matrix (A)
* are 3x3 Identity matrices and the distortion vector (k) and 
* translation vector (t) are zero vectors.
*/

Camera::Camera()
{
	noWorldFrame();
}

Camera::Camera(VideoCapture& camera)
{
	setCamera(camera);
	noWorldFrame();
}

void Camera::setCamera(VideoCapture& camera)
{
	_vc = camera;
}

void Camera::noWorldFrame()
{
	_A = Mat_<double>::eye(A_ROWS, A_COLS);
	_k = Mat_<double>::zeros(K_ROWS, K_COLS);

	_R = Mat_<double>::eye(R_ROWS, R_COLS);
	_t = Mat_<double>::zeros(T_ROWS, T_COLS);
}

/**
* @note No copying is done unless the matrix data is not empty and
* the rows and cols are correct.  If they are not, then no changes
* are made.  The matrix's type does not matter.
*/

void Camera::setA(const Mat& A)
{
	if(!A.empty() && A.rows == A_ROWS && A.cols == A_COLS) {
		A.convertTo(_A, TYPE);
	}
}

/**
* @note No copying is done unless the matrix data is not empty and
* the rows and cols are correct.  If they are not, then no changes
* are made.  The matrix's type does not matter.
*/

void Camera::setK(const Mat& k)
{
	if(!k.empty() && k.rows == K_ROWS && k.cols == K_COLS) {
		k.convertTo(_k, TYPE);
	}
}

/**
* @note No copying is done unless the matrix data is not empty and
* the rows and cols are correct.  If they are not, then no changes
* are made.  The matrix's type does not matter.
*/

void Camera::setR(const Mat& R)
{
	if(!R.empty() && R.rows == R_ROWS && R.cols == R_COLS) {
		R.convertTo(_R, TYPE);
	}
}

/**
* @note No copying is done unless the matrix data is not empty and
* the rows and cols are correct.  If they are not, then no changes
* are made.  The matrix's type does not matter.
*/

void Camera::setT(const Mat& t)
{
	if(!t.empty() && t.rows == T_ROWS && t.cols == T_COLS) {
		t.convertTo(_t, TYPE);
	}
}

/** 
* Converts for pixel to world coordinate frame based on the camera's
* intrinsic and extrinsic parameters.  If these parameters are not set
* an exception is thrown.
*
* @param[in] pixel the pixel location of the dot
* @return the world location of the dot
*/

Point3d Camera::pixelToWorld(const Point2d& pixel) const
{
	double im[2] = {pixel.x, pixel.y};
	double wl[3];
	double z;

	// setup intrinsic and extrinsic parameters
	CvMat img_frm = cvMat(1, 1, CV_64FC2, im);
	Mat world_frm = Mat(T_ROWS, T_COLS, TYPE, wl);

	// convert from distorted pixels to normalized camera frame
	// cv:: version does not allow doubles for some odd reason,
	// so use the C version
	CvMat A = _A;
	CvMat k = _k;
	cvUndistortPoints(&img_frm, &img_frm, &A, &k);

	// convert from camera frame to world frame
	z = _t(2, 0) ? _t(2, 0) : 1;
	wl[0] = z*im[0] - _t(0, 0);
	wl[1] = z*im[1] - _t(1, 0);
	wl[2] = 0;
	world_frm = _R.t() * world_frm;

	return Point3d(wl[0], wl[1], wl[2]);
}

/** 
* Converts for world to pixel coordinate frame based on the camera's
* intrinsic and extrinsic parameters.  If these parameters are not set
* an exception is thrown.
*
* @param[in] world the world location of the dot
* @return the pixel location of the dot
*/

Point2d Camera::worldToPixel(const Point3d& world) const
{
	// all code based on OpenCV cvProjectPoints2 function
	double X, Y, Z, x, y, z;
    double r2, r4, r6, a1, a2, a3, cdist;
    double xd, yd, fx, fy, cx , cy;

	// world frame
	X = world.x;
	Y = world.y;
	Z = 0;

	// camera frame
    x = _R(0, 0)*X + _R(0, 1)*Y + _R(0, 2)*Z + _t(0, 0);
    y = _R(1, 0)*X + _R(1, 1)*Y + _R(1, 2)*Z + _t(1, 0);
    z = _R(2, 0)*X + _R(2, 1)*Y + _R(2, 2)*Z + _t(2, 0);

	// image frame
    z = z ? 1./z : 1;
    x *= z; 
	y *= z;

	fx = _A(0, 0);
	fy = _A(1, 1);
    cx = _A(0, 2);
	cy = _A(1, 2);

    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + _k(0, 0)*r2 + _k(1, 0)*r4 + _k(4, 0)*r6;
    xd = x*cdist + _k(2, 0)*a1 + _k(3, 0)*a2;
    yd = y*cdist + _k(2, 0)*a3 + _k(3, 0)*a1;

	return Point2d(xd*fx + cx, yd*fy + cy);
}

void Camera::mapDots(Dots& dots)
{
	int tag;

	// add the active dots that are in the image
	if(_vc.get(TDAH_PROP_IS_ROI)) {
		// video capture is utilizing an ROI
		tag = static_cast<int> (_vc.get(TDAH_PROP_NEXT_DOT));
		while(tag != NO_MORE_DOTS) {
			dots.makeDotActive(tag);
			tag = static_cast<int> (_vc.get(TDAH_PROP_NEXT_DOT));
		}
	}
	else {
		// assume video capture is returning full image frames
		dots.makeAllDotsActive();
	}
}

/** 	
* Attempts to get frame img_nbr or greater and adds the dots that are
* expected to be captured inside the frame to the active set in preparation
* for a Tracker.
*
* @param[in] img_nbr the image number to grab
* @param[inout] dots the candidate dots that will be added to the active set
* @param[out] img the image associated with the dots
* @return the value is true when an image has been grabbed and dots have
* been added to the active set, otherwise it returns false.
*
* @note if CV_CAP_PROP_POS_FRAMES must work all the time otherwise
* the grab function will resort to using its own internal counter that
* will be out of sync.
*/

bool Camera::grab(int img_nbr, Dots& dots)
{
	int tag;
	double ts;
	ActiveDots a = dots.activeDots();

	// clear all active dots
	dots.clearActiveDots();

	// set the desired image number and grab the image
	if(!_vc.set(CV_CAP_PROP_POS_FRAMES, img_nbr) || !_vc.grab()) {
		return false;
	}

	// get image properties
	ts = _vc.get(CV_CAP_PROP_POS_MSEC);
	img_nbr = static_cast<int> (_vc.get(CV_CAP_PROP_POS_FRAMES));
	mapDots(dots);

	// set image properties
	for(size_t i = 0; i < a.size(); ++i) {
		tag = a[i]->tag();
		dots.timeStamp(tag) = ts;
		dots.imageNbr(tag) = img_nbr;
	}

	return true;
}

bool Camera::grab(Dots& dots)
{
	int tag;
	double ts;
	ActiveDots a = dots.activeDots();
	static int img_nbr = 0;

	// clear all active dots and grab the next image
	dots.clearActiveDots();
	if(!_vc.grab()) {
		return false;
	}

	// update image properties
	++img_nbr;
	ts = cv::getTickCount()/cv::getTickFrequency();
	mapDots(dots);

	// set image properties
	for(size_t i = 0; i < a.size(); ++i) {
		tag = a[i]->tag();
		dots.timeStamp(tag) = ts;
		dots.imageNbr(tag) = img_nbr;
	}

	return true;
}

bool Camera::grab()
{
	return _vc.grab();
}

bool Camera::retrieve(Mat& img, int channel)
{
	return _vc.retrieve(img, channel);
}

/**
* Undistorts the image
*/
void Camera::undistort(Mat& img)
{
	Mat tmp;
	_vc >> tmp;
	cv::undistort(tmp, img, _A, _k);
}