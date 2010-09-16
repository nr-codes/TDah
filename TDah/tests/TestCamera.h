/** @file TestCamera.h
* @brief the test suite for the Camera class
*
* This file is the input to the CxxTest script that is
* used to create the TestCamera.cpp file
*
* @note the TestCamera.cpp file in the project is a place
* holder for the auto-generatated .cpp file from CxxTest
* using its python script.  This is done in V2K8 by running
* a pre-build command in the build event properties setting.
* The command (all on one line) executed is:
 @verbatim
  c:\cygwin\bin\python2.5.exe ../../tests/cxxtest/cxxtestgen.py 
  -o ../../tests/TestCamera.cpp --gui=Win32Gui 
  --runner=ParenPrinter ../../tests/TestCamera.h
 @endverbatim
*
* where the relative directory is assumed to be from where the
* Visual Studio project "Camera.vcproj" is located, which is currently
* ./TDah/build/vs2k8.
*/

#include <cxxtest/TestSuite.h>
#include "Dots.h"
#include "Camera.h"

using cv::Point2f;
using cv::Point3f;
using cv::undistort;

class DummyCamera : public Camera
{
public:
	virtual bool open(const std::string& f) 
	{ 
		img = cv::imread( f );
		return isOpened(); 
	};

	virtual bool isOpened() const { return !img.empty(); };

	virtual bool grab() { return isOpened(); };
	virtual bool retrieve(cv::Mat& image, int ch = 0) 
	{
		img.copyTo(image);
		return isOpened();
	};

	virtual int retrieve(int img_nbr, Dots& dots, cv::Mat& img)
	{
		dots.makeDotActive(0);
		img = this->img;

		return 0;
	};

private:
	cv::Mat img;
};

bool isSameCoord(DummyCamera& cam)
{
	Point3f w1;
	Point2f p1;
	
	// create an arbitrary point
	Point3f w0(32, 7654, 0);

	// and see if we get it back;
	p1 = cam.worldToPixel(w0);
	w1 = cam.pixelToWorld(p1);
	
	return cv::norm(w0 - w1) < 1e-6;
}

class TestCameraSuite : public CxxTest::TestSuite 
{
public:
	void testOpenGrabAndRetrieve( void )
	{
		cv::Mat img;
		DummyCamera cam;
		cam.open("../../tests/TestCamera.bmp");
		TS_ASSERT(cam.isOpened());

		cam >> img;
		TS_ASSERT( !img.empty() );

		cv::namedWindow("image");
		cv::imshow("image", img);
		cv::waitKey(20);
	}

	void testConversion( void )
	{
		Point3f w1;
		Point2f p1;

		// by default cam will use zero vectors for t and k
		// and the identity for R and A.
		// (see doc for definitions of these parameters)
		DummyCamera cam;
		
		// create an arbitrary point
		Point3f w0(74, 29, 0);

		// and see if we get it back
		p1 = cam.worldToPixel(w0);
		w1 = cam.pixelToWorld(p1);
		TS_ASSERT_DELTA(0, cv::norm(w0 - w1), 1e-6);

		// this time can we project back to the pixel plane
		Point2f p0(124, 840);
		w1 = cam.pixelToWorld(p0);
		p1 = cam.worldToPixel(w1);
		TS_ASSERT_DELTA(0, cv::norm(p0 - p1), 1e-6);
	}

	void testChangeToCameraMatrix( void )
	{
		using cv::Mat;
		int n = 3;
		DummyCamera cam;
		Point3f w1;
		Point2f p1;
		cv::Mat A = n * Mat::eye(Camera::A_ROWS, Camera::A_COLS, Camera::TYPE);

		cam.setA(A);

		w1 = Point3f(5, 13, 9);
		p1 = cam.worldToPixel(w1);

		TS_ASSERT_DELTA(p1.x, n*w1.x, 1e-6);
		TS_ASSERT_DELTA(p1.y, n*w1.y, 1e-6);
	}

	void testChangeToRotationMatrix( void )
	{
		DummyCamera cam;
		Point3f w1;
		Point2f p1;
		float swapXY[] = {0, 1, 0, 
						1, 0, 0,
						0, 0, 1};

		// swap x and y coordinates
		cv::Mat R(Camera::R_ROWS, Camera::R_COLS, Camera::TYPE, swapXY);
		cam.setR(R);

		p1 = Point2f(89, 492);

		w1 = cam.pixelToWorld(p1);

		TS_ASSERT_DELTA(p1.x, w1.y, 1e-6);
		TS_ASSERT_DELTA(p1.y, w1.x, 1e-6);
	}

	void testNoChangeToDistortionVector( void )
	{
		Point3f w1;
		Point2f p1;
		cv::Mat k;
		DummyCamera cam;
		
		// test if null matrices are ignored
		cam.setK(k);
		TS_ASSERT( isSameCoord( cam ) );

		// check to see if a regular change makes
		k = cv::Mat::ones(Camera::K_ROWS, Camera::K_COLS, Camera::TYPE);
		cam.setK(k);

		TS_ASSERT( !isSameCoord( cam ) );
	}
};