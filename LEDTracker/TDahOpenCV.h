#ifndef TDAH_OPENCV_H_
#define TDAH_OPENCV_H_

#include "TDah.h"

class TDahOpenCV : public TDah
{
public:
	TDahOpenCV(int index) { is_cap_open = open(index); };
	virtual ~TDahOpenCV() { close(); };
	bool isCapOpen(void) { return is_cap_open; };
	int getROILoc(int index, ROILoc *r);
	void showROILoc(void);

	// CvCapture Inherited Fcns
	virtual bool grabFrame() { return !!cvGrabFrame(cap); };
	virtual bool open( int index ) { return !!(cap = cvCaptureFromCAM(index)); };
	virtual void close() { cvReleaseCapture(&cap); };
	virtual int getCaptureDomain() { return cap->getCaptureDomain(); };
	virtual IplImage* retrieveFrame(int) 
	{ 
		IplImage *img = cvRetrieveFrame(cap);
		cvResetImageROI(img);
		return img;
	};

	virtual double getProperty(int prop) 
	{
		return cvGetCaptureProperty(cap, prop); 
	};

    virtual bool setProperty(int prop, double val) 
	{
		// TODO check to see if bool gets typedcast to int in src code
		return !!cvSetCaptureProperty(cap, prop, val); 
	};

protected:
	bool is_cap_open;
	CvCapture *cap;
};

#endif /* TDAH_OPENCV_H_ */