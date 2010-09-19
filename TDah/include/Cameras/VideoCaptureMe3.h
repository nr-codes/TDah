#ifndef _VIDEOCAPTUREME3_H_
#define _VIDEOCAPTUREME3_H_

#include <vector>
#include <deque>
#include <string>
#include <utility>
#include <cv.h>
#include <highgui.h>
#include <fgrab_struct.h>
#include <FastConfig.h>

#include "_common.h"

class VideoCaptureMe3 : public cv::VideoCapture
{
public:
	static const int REMOVE_ALL = -1;
	enum slots {ROI0 = 0, ROI1, ROI2, ROI3, ROI4, ROI5, ROI6, ROI7};

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
	void add(const Dots& dots);
	void remove(int tag = REMOVE_ALL);
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
	/** @brief contains the dots/ROIs that will be written to the camera */
	std::deque< std::pair< int, cv::Rect> > _q;
	/** @brief local copy of ROIs that have been written to the camera */
	std::vector< std::pair< int, FC_ParameterSet> > _roi;
	/** @brief keeps track of which ROI is in which buffer */
	std::vector< std::pair< int, cv::Rect> > _roi_in_buffer;

	void fastConfigDefaults();
	void me3Err(std::string msg);
	bool roiSequence();
	cv::Rect calcRoi(const cv::Point2d& pixel) const;
	double nextDot();
	bool writeRoi(int slot);
	bool updateRoi();
	static int getTag(int img_tag);
};

#endif /* _VIDEOCAPTUREME3_H_ */