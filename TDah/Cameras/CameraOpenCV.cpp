#include "Dots.h"
#include "Camera.h"
#include "Cameras/CameraOpenCV.h"

using std::string;
using cv::Mat;
using cv::VideoCapture;

CameraOpenCV::CameraOpenCV()
{
	img_nbr = 0;
}

CameraOpenCV::CameraOpenCV(const string& filename) : Camera(filename)
{
	img_nbr = 0;
}

CameraOpenCV::CameraOpenCV(int device) : Camera(device)
{
	img_nbr = 0;
}

void CameraOpenCV::release()
{
	img_nbr = 0;
	VideoCapture::release();
}

/** @brief returns an image, it's number, and adds dots to an active set */
int CameraOpenCV::mapDots(int img_nbr, Dots& dots, Mat& img)
{
	dots.makeAllDotsActive();
	retrieve(img);
	return ++img_nbr;
}
