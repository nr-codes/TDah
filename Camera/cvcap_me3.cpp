#include <_highgui.h>
#include <cv.h>

#include "me3.h"


#if defined(WIN32) && defined(_WIN32)
#define DEFAULT_APPLET FC_APPLET
#endif

#define CV_CAP_me3 -1 // not used
#define NUM_BUFFERS 16
#define MEMSIZE(w, h) ((w) * (h) * NUM_BUFFERS)
#define TIMEOUT 2

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

    virtual bool open(int index);
	virtual bool open(char *applet, int width, int height);
	virtual bool acquire(int *sequence, int seq_len);
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
    IplImage *frame;
	int frame_nr;
};

CvCaptureCAM_me3::CvCaptureCAM_me3()
{

}

void CvCaptureCAM_me3::close()
{
	if(deinit_cam(fg) != FG_OK) {
		me3_err("close");
	}

    cvReleaseImage(&frame);
}

// Initialize camera input
bool CvCaptureCAM_me3::open(int index)
{
	return open(DEFAULT_APPLET, FC_MAX_WIDTH, FC_MAX_HEIGHT);
}

bool CvCaptureCAM_me3::open(char *applet, int w, int h)
{
	if(init_cam(&fg, applet, MEMSIZE(w, h), NUM_BUFFERS) != FG_OK) {
		me3_err("open");
		return false;
	}

	frame = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 1);
	if(frame == NULL) {
		return false;
	}

	return true;
}

bool CvCaptureCAM_me3::acquire(int *sequence, int seq_len)
{
	if(acquire_imgs(fg, sequence, seq_len) != FG_OK) {
		me3_err("acquire");
		return false;
	}

	return true;
}

bool CvCaptureCAM_me3::grabFrame()
{
		img_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
		if(img_nr < FG_OK) {
			me3_err("grabFrame");
			return false;
		}
		
		// get image tag, tag == X => ROI_X
		cur_roi = img_nr;
		rc = Fg_getParameter(fg, FG_IMAGE_TAG, &cur_roi, PORT_A);
		if(rc != FG_OK) {
			printf("loop tag: %s\n", Fg_getLastErrorDescription(fg));
			break;
		}
		
		// get roi associated with image and point to image data
		cur_roi = cur_roi >> 16;
		cur = &(wins.windows[cur_roi]);
		cur->img = (unsigned char *) Fg_getImagePtr(fg, img_nr, PORT_A);




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

double CvCaptureCAM_me3::getProperty(int property_id)
{
	int rc = 0;
	uint64 ts_us;

    switch(property_id) {
		case CV_CAP_PROP_FRAME_COUNT:
			return (double) frame_nr;
		case FG_TIMESTAMP_LONG:
			if(Fg_getParameter(fg, FG_TIMESTAMP_LONG, &ts, PORT_A) == FG_OK) {
				return (double) ts_us;
			}
			break;
		case NUMBER_OF_NEXT_IMAGE:
			rc = Fg_getStatus(fg, NUMBER_OF_NEXT_IMAGE, 0, PORT_A);
			return (double) rc;
		case CV_CAP_PROP_FRAME_WIDTH:
		case FG_WIDTH:
			if(Fg_getParameter(fg, FG_WIDTH, &rc, PORT_A) == FG_OK) {
				return (double) rc;
			}
			break;
		case CV_CAP_PROP_FRAME_HEIGHT:
		case FG_HEIGHT:
			if(Fg_getParameter(fg, FG_HEIGHT, &rc, PORT_A) < 0) {
				return (double) rc;
			}
			break;
		default:
			return -1.0;
    }

	me3_err("grabFrame");
    return -1.0;
}

bool CvCaptureCAM_me3::setProperty(int property_id, double value)
{
    switch(property_id) {
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

CvCapture* cvCreateCameraCapture_me3(char *applet, int w, int h)
{
    CvCaptureCAM_me3 *capture = new CvCaptureCAM_me3;

    if (capture->open(applet, w, h))
        return capture;

    delete capture;
    return NULL;
}

CvCapture* cvCreateCameraCapture_fastconfig()
{
    CvCaptureCAM_me3 *capture = new CvCaptureCAM_me3;

    if (capture->open(FC_APPLET, FC_MAX_WIDTH, FC_MAX_HEIGHT))
        return capture;

    delete capture;
    return NULL;
}


	// make sure that camera returned a valid image
	if(cur->img != NULL) {
		// write ROI position to camera
		write_roi(fg, cur->roi, cur->roi, !DO_INIT);

		// increment to the next desired frame.
		img_nr += NEXT_IMAGE;
	}
	else {
		// typically this state only occurs if an invalid ROI has been programmed
		// into the camera (e.g. roi_w == 4).
		printf("img %d is null\n", img_nr);
		break;
	}