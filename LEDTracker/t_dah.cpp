#include "t_dah.h"
#if 0
#define RETURN_IF_NULL(ptr) do { if(!(ptr)) return cvErrMsg(__FUNCTION__); } while(0)
#define DEFAULT_BUFFERS 16
#define RADIUS_PAD 3
#define MATCH_PAD .03
#define PYR_LVL 2
#define PYR_OFFSET 5

static struct T_Dah {
	ROILoc r;
	IplImage *tplt;
	IplImage *gr;
	CvKalman *kal;
	CvSeqWriter wr;
	CvMemStorage *mem;
	double prev_ts; // TODO: don't forget to initialize prev_ts
} *dots;

static Fg_Struct *fg = NULL;
static CvCapture *capture = NULL;
static IplImage *img, *pyr_temp, *tmplt_temp;
static FC_ParameterSet fc_roi;
static int threshold;
static double min_match, max_radius;
static int n_roi, roi_width, roi_height;


static int allocate_dots(int img_w, int img_h);
int cvErrMsg(char *msg);

int cvErrMsg(char *msg)
{
	printf("%s: (%d) %s\n", 
		msg, cvGetErrStatus(), cvErrorStr(cvGetErrStatus()));

	return cvGetErrStatus();
}

int getROILoc(int roi_nr, ROILoc *roi)
{
	if(roi_nr >= 0 && roi_nr < n_roi) {
		*roi = dots[roi_nr].r;
		return CV_OK;
	}

	return !CV_OK;
}

int init_system(int num_roi, char *yaml_file)
{
	int rc;
	IplImage **tplt = NULL;
	IplImage **gr = NULL;
	CvKalman **kal = NULL;
	CvSeqWriter *wr = NULL;
	CvFileStorage *fs;
	const char *applet;
	double exposure, ft;
	int buffers, index;
	int thresh;
	double match, radius;
	int img_w, img_h, roi_w, roi_h;
	int sequence[] = {ROI_0, ROI_1, ROI_2, ROI_3, ROI_4, ROI_5, ROI_6, ROI_7};

	if(capture || fg) {
		return !CV_OK;
	}

	fs = cvOpenFileStorage(yaml_file, NULL, CV_STORAGE_READ);
	RETURN_IF_NULL(fs);

	read_track_params(fs, &thresh, &match, &radius, &roi_w, &roi_h, 
		&img_w, &img_h);
	if(!thresh || !match || !radius || !roi_w || !roi_h || !img_w || !img_h) {
		return !CV_OK;
	}
	
	n_roi = num_roi;
	roi_width = roi_w;
	roi_height = roi_h;
	min_match = match;
	max_radius = radius;
	threshold = thresh;
	dots = (T_Dah *) cvAlloc(sizeof(T_Dah) * n_roi);
	RETURN_IF_NULL(dots);

	if(read_me3_header(fs, &applet, &exposure, &ft, &buffers) == CV_OK) {
		if(n_roi > MAX_ROI) {
			return !CV_OK; // maybe raise cverror?
		}

		if(buffers == 0) {
			buffers = DEFAULT_BUFFERS;
		}
		if(me3_init(&fg, (char *) applet, img_w*img_h*buffers, buffers) != FG_OK) {
			me3_err("open");
			return !CV_OK;
		}
		if(me3_acquire(fg, sequence, n_roi) != FG_OK) {
			me3_err("acquire");
			return !CV_OK;
		}
	}
	else if(read_opencv_header(fs, &index, &exposure) == CV_OK) {
		capture = cvCaptureFromCAM(index);
		if(exposure) {
			rc = cvSetCaptureProperty(capture, CV_CAP_PROP_EXPOSURE, exposure);
			if(rc != CV_OK) {
				printf("couldn't set exposure\n");
				return rc;
			}
		}
	}
	else {
		return !CV_OK;
	}

	if(allocate_dots(img_w, img_h) != CV_OK) {
		return !CV_OK;
	}

	// maybe not so good to exit if null ptr
	tplt = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	RETURN_IF_NULL(tplt);
	gr = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	RETURN_IF_NULL(gr);
	kal = (CvKalman **) cvAlloc(sizeof(CvKalman *) * n_roi);
	RETURN_IF_NULL(kal);
	wr = (CvSeqWriter *) cvAlloc(sizeof(CvSeqWriter) * n_roi);
	RETURN_IF_NULL(wr);

	for(int i = 0; i < n_roi; i++) {
		tplt[i] = dots[i].tplt;
		gr[i] = dots[i].gr;
		kal[i] = dots[i].kal;
		wr[i] = dots[i].wr;
		dots[i].prev_ts = cvGetTickCount()/cvGetTickFrequency();
		dots[i].r.img_nr = 0;
	}

	setup_kalman(kal, n_roi);
	read_templates(fs, tplt, n_roi);
	read_kalman(fs, kal, n_roi);
	read_obj_loc(fs, gr, n_roi);

	// get dots
	rc = auto_acquire(&queryFrame, gr, roi_width, roi_height, 
		thresh, radius, num_roi, tplt, match, kal, 1);
	if(rc != CV_OK) {
		rc = manual_acquire(&queryFrame, gr, roi_width, roi_height, 
			&thresh, n_roi, tplt, kal);
		threshold = thresh;
	}

	cvFree(&tplt);
	cvFree(&gr);
	cvFree(&kal);
	cvFree(&wr);

	return CV_OK;
}

int deinit_system(void)
{
	for(int i = 0; i < n_roi; i++) {
		cvReleaseImage(&dots[i].gr);
		cvReleaseImage(&dots[i].tplt);

		cvReleaseMemStorage(&dots[i].mem);
		cvReleaseKalman(&dots[i].kal);
	}
	cvFree(&dots);

	cvReleaseImage(&pyr_temp);
	cvReleaseImage(&tmplt_temp);
	cvReleaseImage(&img);
	if(capture) {
		cvReleaseCapture(&capture);
	}
	else if(fg) {
		me3_deinit(fg);
	}
	
	n_roi = 0;
	return CV_OK;
}

static int allocate_dots(int img_w, int img_h)
{
	img = cvCreateImage(cvSize(img_w, img_h), IPL_DEPTH_8U, 1);
	RETURN_IF_NULL(img);
	
	pyr_temp = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	RETURN_IF_NULL(pyr_temp);
	
	tmplt_temp = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
	RETURN_IF_NULL(tmplt_temp);

	for(int i = 0; i < n_roi; i++) {
		dots[i].gr = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		RETURN_IF_NULL(dots[i].gr);
		cvSetImageROI(dots[i].gr, cvRect(0, 0, img->width, img->height));

		dots[i].tplt = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
		RETURN_IF_NULL(dots[i].tplt);
		cvSetImageROI(dots[i].tplt, cvRect(0, 0, img->width, img->height));

		dots[i].kal = cvCreateKalman(X_DIM, Z_DIM, U_DIM);
		RETURN_IF_NULL(dots[i].kal);

		dots[i].mem = cvCreateMemStorage(0);
		RETURN_IF_NULL(dots[i].mem);

		cvStartWriteSeq(CV_SEQ_ELTYPE_POINT | CV_SEQ_KIND_CURVE, 
			sizeof(CvSeq), sizeof(CvPoint), dots[i].mem, &dots[i].wr);
	}

	return CV_OK;
}

static int track_object(int j)
{
	double score;
	float z[Z_DIM];
	CvPoint c;

	cvResetImageROI(img);
	score = track_ctrd(dots[j].gr, roi_width, roi_height, 
		threshold, &dots[j].wr);
	if(!score || score > max_radius) {
		score = track_tmplt_pyr(img, dots[j].tplt, 
			pyr_temp, PYR_LVL, PYR_OFFSET);

		if(score < min_match) {
			score = track_tmplt(img, dots[j].tplt);
			if(score < min_match) {
				 // TODO: Does this get property reset?
				// should filter be used? or should we quit?
				dots[j].r.roi_nr = ~dots[j].r.roi_nr;
				return !CV_OK;
			}
		}

		cvSetImageROI(dots[j].gr, cvGetImageROI(img));
	}

	c = roi2ctrd(dots[j].gr);
	z[0] = (float) c.x;
	z[1] = (float) c.y;
	prediction(dots[j].kal, (float) ((dots[j].r.ts - dots[j].prev_ts)*1e-6), z);
	ctrd2roi(dots[j].gr, 
				cvRound(dots[j].kal->state_post->data.fl[0]), 
				cvRound(dots[j].kal->state_post->data.fl[1]),
				roi_width, 
				roi_height);

	return CV_OK;
}

IplImage *queryFrame()
{
	static int img_nr = 0;

	if(fg) {
		img_nr = queryFrameMe3(img_nr++, NULL);
		return img;
	}
	else if(capture) {
		return cvQueryFrame(capture);
	}

	return NULL;
}

/*************************** OPENCV ***************************/

int init_system_opencv(int index, int roi_w, int roi_h, int num_roi, 
					   char *save_config_as, double exposure)
{
	int rc;
	double val;
	IplImage **tplt = NULL;
	IplImage **gr = NULL;
	CvKalman **kal = NULL;
	CvSeqWriter *wr = NULL;
	CvFileStorage *fs;

	if(capture || fg) {
		return !CV_OK;
	}

	capture = cvCaptureFromCAM(index);
	RETURN_IF_NULL(capture);

	img = cvQueryFrame(capture);
	RETURN_IF_NULL(img);

	n_roi = num_roi;
	roi_width = roi_w;
	roi_height = roi_h;
	dots = (T_Dah *) cvAlloc(sizeof(T_Dah) * n_roi);
	RETURN_IF_NULL(dots);

	if(allocate_dots(img->width, img->height) != CV_OK) {
		return !CV_OK;
	}

	// maybe not so good to exit if null ptr
	tplt = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	RETURN_IF_NULL(tplt);
	gr = (IplImage **) cvAlloc(sizeof(IplImage *) * n_roi);
	RETURN_IF_NULL(gr);
	kal = (CvKalman **) cvAlloc(sizeof(CvKalman *) * n_roi);
	RETURN_IF_NULL(kal);
	wr = (CvSeqWriter *) cvAlloc(sizeof(CvSeqWriter) * n_roi);
	RETURN_IF_NULL(wr);

	for(int i = 0; i < n_roi; i++) {
		tplt[i] = dots[i].tplt;
		gr[i] = dots[i].gr;
		kal[i] = dots[i].kal;
		wr[i] = dots[i].wr;
		dots[i].prev_ts = cvGetTickCount()/cvGetTickFrequency();
		dots[i].r.img_nr = 0;
	}

	setup_kalman(kal, n_roi);
	rc = manual_acquire(&queryFrame, gr, roi_width, roi_height, 
		&threshold, n_roi, tplt, kal);

	max_radius = 0.;
	min_match = 1.;
	IplImage *gr_temp;
	for(int i = 0; i < n_roi; i++) {
		gr_temp = cvCloneImage(gr[i]);

		val = track_ctrd(gr_temp, roi_width, roi_height, threshold, &wr[i]);
		max_radius = MAX(val, max_radius);

		cvCopyImage(gr[i], gr_temp);
		cvResetImageROI(gr_temp);
		val = track_tmplt(gr_temp, tplt[i], tmplt_temp);
		min_match = MIN(val, min_match);
		cvReleaseImage(&gr_temp);
	}

	if(save_config_as) {
		fs = cvOpenFileStorage(save_config_as, NULL, CV_STORAGE_WRITE);
		RETURN_IF_NULL(fs);

		write_opencv_header(fs, index, exposure);
		write_track_params(fs, threshold, min_match - MATCH_PAD, max_radius + RADIUS_PAD, 
			roi_width, roi_height, img->width, img->height);
		write_templates(fs, tplt, n_roi);
		write_obj_loc(fs, gr, n_roi);
		write_kalman(fs, kal, n_roi);
		cvReleaseFileStorage(&fs);
	}

	cvFree(&tplt);
	cvFree(&gr);
	cvFree(&kal);
	cvFree(&wr);

	return CV_OK;
}

int grabFrameOpenCV(int index)
{
	return cvGrabFrame(capture);
}

int retrieveFrameOpenCV(int index, ROILoc *roi)
{
	static int j = 0;
	static int cnt = 0;
	double score;
	float z[Z_DIM];
	CvPoint c;

	cvResetImageROI(img);
	cvCvtColor(cvRetrieveFrame(capture, index), img, CV_BGR2GRAY);

	dots[j].r.roi_nr = j;
	dots[j].r.img_nr = cnt++;
	dots[j].r.ts = cvGetTickCount()/cvGetTickFrequency();

	cvSetImageROI(img, cvGetImageROI(dots[j].gr));
	cvCopyImage(img, dots[j].gr);

	score = track_ctrd(dots[j].gr, roi_width, roi_height, 
		threshold, &dots[j].wr);

	//if(score && score < max_radius) {
		// TODO what should I do here?
		c = roi2ctrd(dots[j].gr);
		z[0] = (float) c.x;
		z[1] = (float) c.y;
		prediction(dots[j].kal, 
			(float) ((dots[j].r.ts - dots[j].prev_ts)*1e-6), z);
		ctrd2roi(dots[j].gr, 
					cvRound(dots[j].kal->state_post->data.fl[0]), 
					cvRound(dots[j].kal->state_post->data.fl[1]),
					roi_width, 
					roi_height);
	//}

	// track_object(j);
	dots[j].prev_ts = dots[j].r.ts;
	dots[j].r.loc = roi2ctrd(dots[j].gr);

	*roi = dots[j].r;

	j++;
	j %= n_roi;

	return cvRound(score);
}

int queryFrameOpenCV(int index, ROILoc *roi)
{
	return grabFrameOpenCV(index) ? 
		retrieveFrameOpenCV(index, roi) : !CV_OK;
}


/*************************** ME3 ***************************/

int grabFrameMe3(int img_nr)
{
	int act_nr;
	int cur_roi, rc;

	act_nr = Fg_getLastPicNumberBlocking(fg, img_nr, PORT_A, TIMEOUT);
	if(act_nr < FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}
	
	// get image tag, tag == X => ROI_X
	cur_roi = act_nr;
	if(Fg_getParameter(fg, FG_IMAGE_TAG, &cur_roi, PORT_A) != FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}
	cur_roi = cur_roi >> 16;

	dots[cur_roi].r.img_nr = act_nr;
	dots[cur_roi].r.ts = act_nr;
	rc = Fg_getParameter(fg, FG_TIMESTAMP_LONG, &dots[cur_roi].r.ts, PORT_A);
	if(rc != FG_OK) {
		me3_err("grabFrame");
		return Fg_getLastErrorNumber(fg);
	}
	
	// get roi associated with image and point to image data
	img->imageData = img->imageDataOrigin = 
		(char *) Fg_getImagePtr(fg, img_nr, PORT_A);

	cvSetImageROI(img, cvGetImageROI(dots[cur_roi].gr));
	cvCopyImage(img, dots[cur_roi].gr);

	return act_nr;
}

int retrieveFrameMe3(int img_nr, ROILoc *roi)
{
	int act_img;
	return CV_OK;
}

int queryFrameMe3(int img_nr, ROILoc *roi)
{ 
	// maybe should return -1 if fallen behind or score
	return grabFrameMe3(img_nr) == img_nr ? retrieveFrameMe3(img_nr, roi) : !CV_OK; 
}
#endif