#ifndef CONFIG_PARSER_H_
#define CONFIG_PARSER_H_

extern void write_track_params(CvFileStorage *fs, int threshold, double min_m,
							   double max_r, int roi_w, int roi_h, 
							   int img_w, int img_h);
extern void write_templates(CvFileStorage *fs, IplImage **tmplt, int n = 1,
							char *dir = ".");
extern void write_obj_loc(CvFileStorage *fs, IplImage **obj, int n = 1);
extern void write_kalman(CvFileStorage *fs, CvKalman **kal, int n = 1);

extern void read_track_params(CvFileStorage *fs, int *threshold, 
						double *min_m, double *max_r, int *roi_w, int *roi_h,
						int *img_w, int *img_h);
extern void read_templates(CvFileStorage *fs, IplImage **tmplt, int n = 1);
extern void read_obj_loc(CvFileStorage *fs, IplImage **obj, int n = 1);
extern void read_kalman(CvFileStorage *fs, CvKalman **kal, int n = 1);

#endif /* CONFIG_PARSER_H_ */
