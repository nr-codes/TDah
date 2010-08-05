#ifndef _DOT_COORD_H_
#define _DOT_COORD_H_

extern CvPoint2D32f pixel2world(CvPoint p, CvMat *A, CvMat *k, 
								CvMat *R, CvMat *T);
extern CvPoint world2pixel(CvPoint2D32f w, CvMat *A, CvMat *k, 
						   CvMat *R, CvMat *T);

extern int grab_calib_grid(CvCapture *capture, CvSize grid_size, 
					CvMat *ip, CvMat *op, CvMat *pc, int num_imgs, 
					int prompt = false, IplImage *store_imgs[] = NULL);

extern int get_camera_intrinsics(CvCapture *capture, char *save_file, int rows, 
								 int cols, int num_images);

extern int get_camera_extrinsics(CvCapture *capture, char *save_file, 
								 char *intrinsic_file, int rows, int cols, 
								 double scale = 1,
								 CvPoint2D32f origin = cvPoint2D32f(0, 0), 
								 float theta = 0);

#endif /* _DOT_COORD_H_ */