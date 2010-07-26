#include "_highgui.h"
#include <cv.h>

#include "fgrab_struct.h"
#include "fgrab_prototyp.h"
#include "fgrab_define.h"
#include "FastConfig.h"

#if defined(WIN32) && defined(_WIN32)
#define DEFAULT_APPLET "FastConfig.dll"
#define FAST_CONFIG "FastConfig.dll"
#endif

#define CV_CAP_me3 -1 // not used
#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define NUM_BUFFERS 16
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)
#define TIMEOUT 3

#define me3_err(msg) printf("me3::%s: (%d) %s\n", (msg), Fg_getLastErrorNumber(fg), \
							Fg_getLastErrorDescription(fg));

/********************* Capturing video from camera via PvAPI *********************/

class CvCaptureCAM_me3 : public CvCapture
{
public:
    CvCaptureCAM_me3();
    virtual ~CvCaptureCAM_me3() 
    {
        close();
    }

    virtual bool open( int index );
	virtual bool open( char *applet );
    virtual void close();
    virtual double getProperty(int);
    virtual bool setProperty(int, double);
    virtual bool grabFrame();
    virtual IplImage* retrieveFrame(int);
    virtual int getCaptureDomain() 
    {
        return CV_CAP_me3;
    }

protected:
	Fg_Struct *fg;
	const unsigned long *mem;
    IplImage *frame;
	int frame_nr;
	bool fast_config;
};


CvCaptureCAM_me3::CvCaptureCAM_me3()
{

}

void CvCaptureCAM_me3::close()
{
	int rc;

	rc = Fg_stopAcquire(fg, PORT_A);
	if(rc != FG_OK) {
		me3_err("close");
	}

	rc = Fg_FreeMem(fg, PORT_A);
	if(rc != FG_OK) {
		me3_err("close");
	}

	if(fast_config) {
		rc = FastConfigFree(PORT_A);
		if(rc != FG_OK) {
			me3_err("close");
		}
	}

	rc = Fg_FreeGrabber(fg);
	if(rc != FG_OK) {
		me3_err("close");
	}

    cvReleaseImage(&frame);
}

// Initialize camera input
bool CvCaptureCAM_me3::open(int index)
{
	return open(DEFAULT_APPLET);
}

bool CvCaptureCAM_me3::open(char *applet)
{
	// int init_cam(Fg_Struct **grabber, int memsize, int buffers, int camlink)
	int rc, camlink;

	frame_nr = 1;
	frame = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U, 1);
	if(frame == NULL) {
		printf("me3::open couldn't allocate image\n");
		return false;
	}

	fg = Fg_Init(applet, PORT_A);
	if(fg == NULL) {
		me3_err("open");
		return false;
	}

	camlink = FG_CL_DUALTAP_8_BIT;
	rc = Fg_setParameter(fg, FG_CAMERA_LINK_CAMTYP, &camlink, PORT_A);
	if(rc != FG_OK) {
		me3_err("open");
		return false;
	}

	fast_config = !strcmp(applet, FAST_CONFIG);
	if(fast_config) {
		rc = FastConfigInit(PORT_A);
		if(rc != FG_OK) {
			me3_err("open");
			return false;
		}
	}

	mem = Fg_AllocMem(fg, MEMSIZE(IMG_WIDTH, IMG_HEIGHT), NUM_BUFFERS, PORT_A);
	if(mem == NULL) {
		me3_err("open");
		return false;
	}

	rc = Fg_Acquire(fg, PORT_A, GRAB_INFINITE);
	if(rc != FG_OK){ 
		me3_err("open");
		return false;
	}

	return true;
}

bool CvCaptureCAM_me3::grabFrame()
{
	frame_nr = Fg_getLastPicNumberBlocking(fg, frame_nr, PORT_A, TIMEOUT);
	if(frame_nr < FG_OK) {
		me3_err("grabFrame");
		return false;
	}

	// TODO: make work with ROI
	cvSetData(frame, 
		Fg_getImagePtr(fg, frame_nr, PORT_A), 
		CV_AUTOSTEP);

    return true;
}

IplImage* CvCaptureCAM_me3::retrieveFrame(int index)
{
	return frame;
}

double CvCaptureCAM_me3::getProperty( int property_id )
{
    switch (property_id)
    {
		case CV_CAP_PROP_FRAME_WIDTH:
			return (double) IMG_WIDTH;
		case CV_CAP_PROP_FRAME_HEIGHT:
			return (double) IMG_HEIGHT; // TODO: make queryable
    }

    return -1.0;
}

bool CvCaptureCAM_me3::setProperty( int property_id, double value )
{
    switch ( property_id )
    {
    // TODO: Camera works, but IplImage must be modified for the new size
    case CV_CAP_PROP_FRAME_WIDTH:
    case CV_CAP_PROP_FRAME_HEIGHT:
    default:
        return false;
    }
    return true;
}


CvCapture* cvCreateCameraCapture_me3(int index)
{
    CvCaptureCAM_me3 *capture = new CvCaptureCAM_me3;

    if (capture->open(index))
        return capture;

    delete capture;
    return NULL;
}

CvCapture* cvCreateCameraCapture_me3(char *applet)
{
    CvCaptureCAM_me3 *capture = new CvCaptureCAM_me3;

    if (capture->open(applet))
        return capture;

    delete capture;
    return NULL;
}