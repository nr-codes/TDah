#if 0

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
#endif