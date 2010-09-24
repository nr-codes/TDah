#include <fcdynamic.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include "grab.h"

#define IMG_WIDTH 1024
#define IMG_HEIGHT 1024
#define EXPOSURE 20000
#define FRAME_TIME 50000
#define TIMEOUT 5
#define SEQ_LEN 1

static CvCapture *capture = NULL;
static IplImage *img;
static Fg_Struct *fg = NULL;
static TrackingSequence tseq;

static void init_roi0(TrackingWindow *win, int roi_box, double frame, double exposure);

int init_cam_src(int src)
{
	if(src == FRAME_GRABBER) {
		img = cvCreateImage(cvSize(IMG_WIDTH, IMG_HEIGHT), IPL_DEPTH_8U , 1);
		tseq.seq = new int[SEQ_LEN];
		tseq.seq[0] = ROI_0;
		tseq.seq_len = SEQ_LEN;
		init_roi0(tseq.windows, IMG_WIDTH, FRAME_TIME, EXPOSURE);
		if(StartGrabbing(&fg, &tseq, NULL) != FG_OK) {
			printf("init_cam_src: can't open camera\n");
			return -2;
		}
	}
	else if(src == WEBCAM) {
		capture = cvCaptureFromCAM(0);
		if(!capture) {
			printf("init_cam_src: can't open camera\n");
			return -1;
		}
	}

	return 0;
}

int close_cam_src(int src)
{
	if(src == FRAME_GRABBER) {
		cvReleaseImage(&img);
		if(deinit_cam(fg) != FG_OK) {
			printf("deinit: %s\n", Fg_getLastErrorDescription(fg));
			return -2;
		}
	}
	else if(src == WEBCAM) {
		cvReleaseCapture(&capture);
	}

	return 0;
}

// returned image should not be freed by user 
// nor should image header be modified
IplImage *grab(int src)
{
	if(src == FRAME_GRABBER) {
		tseq.windows[ROI_0].img = (unsigned char *) Fg_getImagePtr(fg, 
			Fg_getImage(fg, SEL_NEW_IMAGE, 0, PORT_A, TIMEOUT), PORT_A);
		if(tseq.windows[ROI_0].img != NULL) {
			CopyTrackingWindowToImage(tseq.windows, img);
			return img;
		}
		// otherwise return NULL
	}
	else if(src == WEBCAM) {
		return cvQueryFrame(capture);
	}

	return NULL;
}

int grab4calib(IMAGE_SOURCE src, CvMat *op, CvMat *ip, CvMat* pc, CvSize *grid_size, 
			   int num_imgs, int prompt, IplImage *store_imgs[])
{
	int rc, input, rows = grid_size->height, cols = grid_size->width;
	int num_points = rows * cols;
	IplImage *img = NULL, *gray = NULL, *rgb = NULL;
	CvPoint2D32f *corners;
	int corners_found = 0, good_imgs = 0;

	rc = init_cam_src(src);
	if(rc != FG_OK) {
		return -39;
	}

	img = grab(src);
	if(img == NULL) {
		close_cam_src(src);
		return -1;
	}
	gray = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);
	rgb = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);

	corners = (CvPoint2D32f *) malloc(sizeof(CvPoint2D32f) * num_points);
	if(corners == NULL) {
		close_cam_src(src);
		return -ENOMEM;
	}

	cvNamedWindow("calibration", CV_WINDOW_AUTOSIZE);
	while(good_imgs < num_imgs) {
		img = grab(src);
		if(img != NULL) {
			if(src == FRAME_GRABBER) {
				cvCopyImage(img, gray);
				cvCvtColor(gray, rgb, CV_GRAY2BGR);
			}
			else if(src == WEBCAM) {
				cvCopyImage(img, rgb);
				cvCvtColor(rgb, gray, CV_BGR2GRAY);
			}

			rc = cvFindChessboardCorners(gray, *grid_size, corners, &corners_found);
			if(rc) {
				cvFindCornerSubPix(gray, corners, corners_found, cvSize(5, 5), 
					cvSize(-1, -1), 
					cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
				
				//store pixel points
				CV_MAT_ELEM(*pc, int, good_imgs, 0) = corners_found;
				for(int i = 0; i < num_points; i++) {
					CV_MAT_ELEM(*ip, float, good_imgs*num_points + i, 0) = corners[i].x;
					CV_MAT_ELEM(*ip, float, good_imgs*num_points + i, 1) = corners[i].y;
					CV_MAT_ELEM(*op, float, good_imgs*num_points + i, 0) = i/cols;
					CV_MAT_ELEM(*op, float, good_imgs*num_points + i, 1) = i%cols;
					CV_MAT_ELEM(*op, float, good_imgs*num_points + i, 2) = 0;
				}

				if(store_imgs != NULL) {
					store_imgs[good_imgs] = cvCloneImage(rgb);
				}

				good_imgs++;
			}

			cvDrawChessboardCorners(rgb, *grid_size, corners, corners_found, rc);
			cvShowImage("calibration", rgb);

			if(rc && prompt) {
				// prompt user to keep results or overwrite it
				input = cvWaitKey(0);
				if(input == 'i') {
					good_imgs--;
				}
			}
			else {
				input = cvWaitKey(1);
				if(input == 'q') {
					break;
				}
			}
		}
		else {
			printf("img is null\n");
			break;
		}
	}

	close_cam_src(src);
	cvReleaseImage(&gray);
	cvReleaseImage(&rgb);
	free(corners);

	return good_imgs;
}


//code taken from: http://blog.weisu.org/2007/11/opencv-print-matrix.html
// last accessed 06/24/10
void PrintMat(CvMat *A)
{
	int i, j;
	for (i = 0; i < A->rows; i++) {
		printf("\n");
		switch (CV_MAT_DEPTH(A->type)) {
			case CV_32F:
			case CV_64F:
				for (j = 0; j < A->cols; j++) {
					printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
				}
				break;
			case CV_8U:
			case CV_16U:
				for(j = 0; j < A->cols; j++) {
					printf ("%6d",(int)cvGetReal2D(A, i, j));
				}
				break;
			default:
				break;
		}
	}
	printf("\n");
}

// taken from OpenCV sample code calibration.cpp
double compute_reprojection_error( const CvMat* object_points,
        const CvMat* rot_vects, const CvMat* trans_vects,
        const CvMat* camera_matrix, const CvMat* dist_coeffs,
        const CvMat* image_points, const CvMat* point_counts,
        CvMat* per_view_errors )
{
    CvMat* image_points2 = cvCreateMat( image_points->rows,
        image_points->cols, image_points->type );
    int i, image_count = rot_vects->rows, points_so_far = 0;
    double total_err = 0, err;
    
    for( i = 0; i < image_count; i++ )
    {
        CvMat object_points_i, image_points_i, image_points2_i;
        int point_count = point_counts->data.i[i];
        CvMat rot_vect, trans_vect;

        cvGetRows( object_points, &object_points_i,
            points_so_far, points_so_far + point_count );
        cvGetRows( image_points, &image_points_i,
            points_so_far, points_so_far + point_count );
        cvGetRows( image_points2, &image_points2_i,
            points_so_far, points_so_far + point_count );
        points_so_far += point_count;

        cvGetRow( rot_vects, &rot_vect, i );
        cvGetRow( trans_vects, &trans_vect, i );

        cvProjectPoints2( &object_points_i, &rot_vect, &trans_vect,
                          camera_matrix, dist_coeffs, &image_points2_i,
                          0, 0, 0, 0, 0 );
        err = cvNorm( &image_points_i, &image_points2_i, CV_L1 );
        if( per_view_errors )
            per_view_errors->data.db[i] = err/point_count;
        total_err += err;
    }
    
    cvReleaseMat( &image_points2 );
    return total_err/points_so_far;
}

static void init_roi0(TrackingWindow *win, int roi_box, double frame, double exposure)
{
	memset(win, 0, sizeof(TrackingWindow) * MAX_ROI);
	win[ROI_0].blob_xmin = 0;
	win[ROI_0].blob_ymin = 0;
	win[ROI_0].blob_xmax = IMG_WIDTH;
	win[ROI_0].blob_ymax = IMG_HEIGHT;
	win[ROI_0].roi = ROI_0;
	win[ROI_0].roi_w = roi_box;
	win[ROI_0].roi_h = roi_box;
	win[ROI_0].img_w = IMG_WIDTH;
	win[ROI_0].img_h = IMG_HEIGHT;
	set_roi_box(win, IMG_WIDTH/2, IMG_HEIGHT/2);
	fix_blob_bounds(win);
	SetTrackCamParameters(win, frame, exposure);
}
