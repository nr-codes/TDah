#ifndef _VIDEOCAPTUREME3_H_
#define _VIDEOCAPTUREME3_H_

#include <vector>
#include <queue>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <fgrab_struct.h>
#include <FastConfig.h>

#include "_common.h"

class VideoCaptureMe3 : public cv::VideoCapture
{
public:
	enum slots {ROI_0 = 0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

    VideoCaptureMe3();
	VideoCaptureMe3(const std::string& filename);
    VideoCaptureMe3(int device);
    
    ~VideoCaptureMe3();
	bool open(const std::string& filename);
    bool open(int device);
    bool isOpened() const;
    void release();
    
    bool grab();
	bool retrieve(cv::Mat& image, int channel=0);
	//virtual VideoCaptureMe3& operator >> (cv::Mat& image);
    
    bool set(int prop, double value);
    double get(int prop);

	// functions not in cv::VideoCapture
	bool buffers(int n, int width, int height);
	bool start(int n = GRAB_INFINITE);
	bool stop();
	void enqueue(const Dots& dots);
	bool setRois(Dots& dots, cv::Size roi, double exposure, double fps);

	static void makeSafeMat(cv::Mat& mat);
	static void makeUnsafeMat(cv::Mat& mat, cv::Point& offset);
    
private:
	int _img_nbr;
	int _buffers;
	int _tap;
	int _trigger;
	uchar* _mem;
	Fg_Struct* _fg;
	FC_ParameterSet _roi;
	std::vector< std::pair< int, cv::Rect> > _r;
	std::queue< std::pair< int, cv::Rect> > _q;

	void fastConfigDefaults();
	void me3Err(std::string msg);
	bool roiSequence();
	cv::Rect calcRoi(const cv::Point2d& pixel) const;
	double nextDot();
};

#endif /* _VIDEOCAPTUREME3_H_ */