#ifndef _DOT_COORD_H_
#define _DOT_COORD_H_

extern int grab_calib_grid(CvCapture *capture, CvSize grid_size, 
					CvMat *ip, CvMat *op, CvMat *pc, int num_imgs, 
					int prompt = false, IplImage *store_imgs[] = NULL);

extern int get_camera_intrinsics(CvCapture *capture, 
								 int rows, int cols, int num_images, 
								 CvPoint2D32f origin = cvPoint2D32f(0, 0), 
								 float theta = 0);

#endif /* _DOT_COORD_H_ */