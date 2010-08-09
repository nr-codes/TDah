#ifndef DRAW_DOTS_H_
#define DRAW_DOTS_H_

extern void show_position(IplImage **roiImg, int n, 
				   CvKalman **kal = NULL, CvSeqWriter *bndry = NULL, 
				   char *draw_what = NULL, IplImage *fullImg = NULL);

extern void show_tplts(IplImage **tplt, int roi_w, int roi_h, 
					   int rows, int cols, int n);
extern void show_seqs(CvSeqWriter *bndry, int roi_w, int roi_h, 
					  int rows, int cols, int n);
extern void show_undistorted_image(IplImage *img, CvMat *A, CvMat *k);
extern void draw_ctrd(IplImage *dst, IplImage *src, CvSeq *bndry, int i);
extern void draw_kal(IplImage *dst, CvKalman *kal);
extern void draw_axis(IplImage *dst, char *axis_label, CvPoint org, 
					  CvMat *R = NULL);
extern void draw_wrld2pxl(IplImage *dst, int rows, int cols, 
						  CvMat *wrld_pts, CvMat *prj_wrld_pts, CvMat *img_pts);
extern void draw_wrld2pxl(IplImage *dst, char *axis, CvMat *A, CvMat *k, 
						  CvMat *R, CvMat *t);

#endif /* DRAW_DOTS_H_ */
