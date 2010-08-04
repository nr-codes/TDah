#include "t_dah.h"

#define DEFAULT_MATCH 1
#define DEFAULT_VALUE 0

#define TRACK_STRUCT "Tracking Parameters"
#define THRESH_KEY "Threshold Value"
#define MATCH_KEY "Min Template Match"
#define RADIUS_KEY "Max Radius"
#define ROIW_KEY "Software ROI Width"
#define ROIH_KEY "Software ROI Height"
#define IMGW_KEY "Camera Image Width"
#define IMGH_KEY "Camera Image Height"

#define TMPLT_STRUCT "Object Templates"
#define TMPLT_FILE_FMT "/tmplt%d.png"
#define TMPLT_KEY_FMT "Template%d"

#define ROILOC_STRUCT "Initial Object Rectangle"
#define ROILOC_KEY_FMT "Object%d"
#define ROILOC_XPOS "X Position"
#define ROILOC_YPOS "Y Position"
#define ROILOC_W "Box Width"
#define ROILOC_H "Box Height"

#define KAL_STRUCT "Initial Conditions for Kalman Filter"
#define KAL_STATE_FMT "KalmanState%d"
#define KAL_COV_FMT "KalmanCov%d"

void write_track_params(CvFileStorage *fs, int threshold, 
						double min_m, double max_r, int roi_w, int roi_h,
						int img_w, int img_h)
{
	cvStartWriteStruct(fs, TRACK_STRUCT, CV_NODE_MAP);
	cvWriteInt(fs, THRESH_KEY, threshold);
	cvWriteReal(fs, MATCH_KEY, min_m);
	cvWriteReal(fs, RADIUS_KEY, max_r);
	cvWriteInt(fs, ROIW_KEY, roi_w);
	cvWriteInt(fs, ROIH_KEY, roi_h);
	cvWriteInt(fs, IMGW_KEY, img_w);
	cvWriteInt(fs, IMGH_KEY, img_h);
	cvEndWriteStruct(fs);
}

void write_templates(CvFileStorage *fs, IplImage **tmplt, int n, char *dir)
{
	char path[500];
	char key[100];
	int len;

	if(!tmplt) return;
	
	memset(path, 0, sizeof(path));
	len = snprintf(path, 50, "%s", dir);
	cvStartWriteStruct(fs, TMPLT_STRUCT, CV_NODE_MAP);
	for(int i = 0; i < n; i++) {
		if(tmplt[i]) {
			memset(path + len, 0, sizeof(path) - len);
			snprintf(path + len, sizeof(path) - len, TMPLT_FILE_FMT, i);
			cvSaveImage(path, tmplt[i]);

			memset(key, 0, sizeof(key));
			snprintf(key, sizeof(key), TMPLT_KEY_FMT, i);
			cvWriteString(fs, key, path);
		}
	}
	cvEndWriteStruct(fs);
}

void write_obj_loc(CvFileStorage *fs, IplImage **obj, int n)
{
	char key[100];
	CvRect rect;

	if(!obj) return;

	cvStartWriteStruct(fs, ROILOC_STRUCT, CV_NODE_MAP);
	for(int i = 0; i < n; i++) {
		if(obj[i]->roi) {
			memset(key, 0, sizeof(key));
			snprintf(key, sizeof(key), ROILOC_KEY_FMT, i);
			rect = cvGetImageROI(obj[i]);

			cvStartWriteStruct(fs, key, CV_NODE_MAP);
			cvWriteInt(fs, ROILOC_XPOS, rect.x);
			cvWriteInt(fs, ROILOC_YPOS, rect.y);
			cvWriteInt(fs, ROILOC_W, rect.width);
			cvWriteInt(fs, ROILOC_H, rect.height);
			cvEndWriteStruct(fs);
		}
	}
	cvEndWriteStruct(fs);
}

void write_kalman(CvFileStorage *fs, CvKalman **kal, int n)
{
	char key[100];

	if(!kal) return;

	cvStartWriteStruct(fs, KAL_STRUCT, CV_NODE_MAP);
	for(int i = 0; i < n; i++) {
		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), KAL_STATE_FMT, i);
		cvWrite(fs, key, kal[i]->state_post);

		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), KAL_COV_FMT, i);
		cvWrite(fs, key, kal[i]->error_cov_post);
	}
	cvEndWriteStruct(fs);
}

///////////////////////////

void read_track_params(CvFileStorage *fs, int *threshold, 
						double *min_m, double *max_r, int *roi_w, int *roi_h,
						int *img_w, int *img_h)
{
	CvFileNode *node = cvGetFileNodeByName(fs, NULL, TRACK_STRUCT);

	*threshold = cvReadIntByName(fs, node, THRESH_KEY, DEFAULT_VALUE);
	*min_m = cvReadRealByName(fs, node, MATCH_KEY, DEFAULT_MATCH);
	*max_r = cvReadRealByName(fs, node, RADIUS_KEY, DEFAULT_VALUE);
	*roi_w = cvReadIntByName(fs, node, ROIW_KEY, DEFAULT_VALUE);
	*roi_h = cvReadIntByName(fs, node, ROIH_KEY, DEFAULT_VALUE);
	*img_w = cvReadIntByName(fs, node, IMGW_KEY, DEFAULT_VALUE);
	*img_h = cvReadIntByName(fs, node, IMGH_KEY, DEFAULT_VALUE);
}

void read_templates(CvFileStorage *fs, IplImage **tmplt, int n)
{
	const char *img_path;
	char key[100];
	IplImage *img;
	CvFileNode *node = cvGetFileNodeByName(fs, NULL, TMPLT_STRUCT);

	if(node == NULL) {
		return;
	}

	for(int i = 0; i < n; i++) {
		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), TMPLT_KEY_FMT, i);
		img_path = cvReadStringByName(fs, node, key, NULL);
		if(img_path != NULL) {
			img = cvLoadImage(img_path, CV_LOAD_IMAGE_GRAYSCALE);
			cvSetImageROI(tmplt[i], cvGetImageROI(img));
			cvCopyImage(img, tmplt[i]);
			cvReleaseImage(&img);
		}
	}
}

void read_obj_loc(CvFileStorage *fs, IplImage **obj, int n)
{
	char key[100];
	CvRect rect;
	CvFileNode *node = cvGetFileNodeByName(fs, NULL, ROILOC_STRUCT);
	CvFileNode *obj_node;

	for(int i = 0; i < n; i++) {
		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), ROILOC_KEY_FMT, i);
		obj_node = cvGetFileNodeByName(fs, node, key);

		rect.x = cvReadIntByName(fs, obj_node, ROILOC_XPOS, DEFAULT_VALUE);
		rect.y = cvReadIntByName(fs, obj_node, ROILOC_YPOS, DEFAULT_VALUE);
		rect.width = cvReadIntByName(fs, obj_node, ROILOC_W, obj[i]->width);
		rect.height = cvReadIntByName(fs, obj_node, ROILOC_H, obj[i]->height);
		cvSetImageROI(obj[i], rect);
	}
}

void read_kalman(CvFileStorage *fs, CvKalman **kal, int n)
{
	char key[100];
	CvMat *mat;
	CvFileNode *node = cvGetFileNodeByName(fs, NULL, KAL_STRUCT);

	if(node == NULL) {
		return;
	}

	for(int i = 0; i < n; i++) {
		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), KAL_STATE_FMT, i);
		mat = (CvMat *) cvReadByName(fs, node, key);
		if(mat != NULL) {
			cvCopy(mat, kal[i]->state_post);
		}

		memset(key, 0, sizeof(key));
		snprintf(key, sizeof(key), KAL_COV_FMT, i);
		mat = (CvMat *) cvReadByName(fs, node, key);
		if(mat != NULL) {
			cvCopy(mat, kal[i]->error_cov_post);
		}
	}
}
