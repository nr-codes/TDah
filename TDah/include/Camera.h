#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <vector>
#include <cv.h>
#include <highgui.h>
#include "_common.h"

/**
* @brief A class for retrieving images and performing camera calibration.
*
* The Camera class provides basic functionality for taking images and
* converting between the world and pixel coordinate frames.
* 
*/

class Camera
{
public:
	static const int TYPE = CV_64FC1;
	static const int A_ROWS = 3; /**< number of rows for camera matrix */
	static const int A_COLS = 3; /**< number of cols for camera matrix */
	static const int R_ROWS = 3; /**< number of rows for rotation matrix */
	static const int R_COLS = 3; /**< number of cols for rotation matrix */
	static const int T_ROWS = 3; /**< number of rows for translation vector */
	static const int T_COLS = 1; /**< number of cols for translation vector */
	static const int K_ROWS = 5; /**< number of rows for distortion vector */
	static const int K_COLS = 1; /**< number of cols for distortion vector */

	/** @name Constructors */
	//@{
	Camera();
	Camera(cv::VideoCapture& vc);
	//@}

	/** @brief sets a new camera */
	void setCamera(cv::VideoCapture& vc);

	/** @brief sets the A_ROWS x A_COLS camera matrix */
	void setA(const cv::Mat& A);
	/** @brief sets the K_ROWS distortion coefficent column vector */
	void setK(const cv::Mat& k);
	/** @brief sets the R_ROWS x R_COLS world to pixel rotation matrix */
	void setR(const cv::Mat& R);
	/** @brief sets the world to pixel translation column vector */
	void setT(const cv::Mat& t);
	/**@brief sets pixel and world frame equal to each other */
	void noWorldFrame();

	/** @brief converts from pixel to world points */
	cv::Point3d pixelToWorld(const cv::Point2d& pixel) const;
	/** @brief converts from world to pixel points */
	cv::Point2d worldToPixel(const cv::Point3d& world) const;

	/** @brief requests the next image and adds dots to the active set */
	bool grab(Dots& dots);
	/** @brief sets an image, it's number, and adds dots to an active set */
	bool grab(int img_nbr, Dots& dots);
	
	/** @brief calls the underlying camera's grab function */
	bool grab();
	/** @brief calls the underlying camera's retrieve function */
	bool retrieve(cv::Mat& img, int channel=0);

	/** @brief undistorts an image */
	void undistort(cv::Mat& img);
cv::VideoCapture* _vc; // TODO DELETE
private:
	/** @brief the actual camera */
	//cv::VideoCapture* _vc; // TODO UNCOMMENT
	/** @brief camera matrix */
	cv::Mat_<double> _A;
	/** @brief distortion coefficients vector */
	cv::Mat_<double> _k;
	/** @brief world to camera frame rotation matrix */
	cv::Mat_<double> _R;
	/** @brief world to camera frame translation vector */
	cv::Mat_<double> _t;

	/** @brief maps dots to an image */
	void mapDots(Dots& dots);
};

#endif /* _CAMERA_H_ */