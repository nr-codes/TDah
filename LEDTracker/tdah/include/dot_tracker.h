#ifndef DOT_TRACKER_H_
#define DOT_TRACKER_H_

/**
* Tracks a circular dot in a grayscale image.
*
* This function takes an 8-bit 1- or 3-channel image, binarizes the image, and 
* returns the best-fit circle based on a subset of boundary pixels.  The 
* subset of boundary pixels are defined by a 3x1 kernel centered at the pixel
* under consideration.  If the pixel is <code> WHITE </code> and exactly one 
* other pixel in the kernel is also WHITE, then the pixel is considered to be 
* on the boundary of the dot.  The functions modifies the image's ROI and image
* data.  The ROI is updated to be centered around the center of the circle in
* image frame and the image is binarized based on the threshold value.
*
* @param gray an 8-bit grayscale image, the ROI MUST be set
* @param roi_w the desired ROI width to use when centering the updated ROI
* @param roi_h the region of interest height
* @param thresh the image threshold value
* @param wr the sequence writer of (x,y) coordinates for fast point storage
*/
extern double track_ctrd(IplImage *gray, int roi_w, int roi_h, 
						 int thresh, int thresh_type, CvSeqWriter *wr, 
						 CvPoint2D32f *ctrd);

extern double track_tmplt(IplImage *gray, IplImage *templ, 
						  IplImage *temp = NULL);

extern double track_tmplt_pyr(IplImage *gr, IplImage *tmplt, 
					   IplImage *temp, int lvl = 1, int offset = 0);

#endif /* DOT_TRACKER_H_ */
