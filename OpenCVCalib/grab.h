#ifndef GRAB_H_
#define GRAB_H_

enum IMAGE_SOURCE {FRAME_GRABBER, WEBCAM, LOCAL_FILE};

extern int close_cam_src(int src = FRAME_GRABBER);
extern int init_cam_src(int src = FRAME_GRABBER);
extern IplImage *grab(int src = FRAME_GRABBER);
extern int grab4calib(IMAGE_SOURCE src, CvMat *op, CvMat *ip, CvMat* pc, 
					  CvSize *grid_size, int num_imgs, int prompt = 0, 
					  IplImage *store_imgs[] = NULL);
extern void PrintMat(CvMat *A);
extern double compute_reprojection_error( const CvMat* object_points,
        const CvMat* rot_vects, const CvMat* trans_vects,
        const CvMat* camera_matrix, const CvMat* dist_coeffs,
        const CvMat* image_points, const CvMat* point_counts,
        CvMat* per_view_errors );

#endif /* GRAB_H_ */