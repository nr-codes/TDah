/**
* @file imgproc.cpp a collection of a few image processing routines
*/

#include "fcdynamic.h"

/**
* binarizes an image
*
* <code>threshold</code> takes a TrackingWindow and binarizes the data based on the 
* threshold level, <code>t</code>.  All values less than <code>t</code> are colored as
* <code>BACKGROUND</code> pixels, otherwise the pixel is a <code>FOREGROUND</code> pixel.
*
* @param win the TrackingWindow to threshold
* @param t the threshold value
*/

int threshold(TrackingWindow *win, int t)
{
	int i, j, xmax, ymax;

	xmax = win->blob_xmax;
	ymax = win->blob_ymax;

	for(i = win->blob_ymin; i < ymax; i++) {
		for(j = win->blob_xmin; j < xmax; j++) {
#if FOREGROUND == WHITE
			PIXEL(win, i, j) = (PIXEL(win, i, j) < t) ? BACKGROUND : FOREGROUND;
#else
			PIXEL(win, i, j) = (PIXEL(win, i, j) > t) ? BACKGROUND : FOREGROUND;
#endif
		}
	}

	return 0;
}

/**
* finds the boundary of an image
*
* <code>boundary</code> takes a TrackingWindow after the image data has been binarized 
* and finds the boundary of the object
*
* @param win the TrackingWindow with the binarized image data
*/

int boundary(TrackingWindow *win)
{
	int i, j, xmax, ymax;

	xmax = win->blob_xmax;
	ymax = win->blob_ymax;

	for(i = win->blob_ymin; i < ymax; i++) {
		for(j = win->blob_xmin; j < xmax; j++) {
			if( // center pixel
				PIXEL(win, i, j) == FOREGROUND &&
				// upper neighbor
				((i - 1 < 0) || PIXEL(win, i - 1, j) == BACKGROUND ||
				// lower neighbor
				(i > ymax - 2) || PIXEL(win, i + 1, j) == BACKGROUND ||
				// left neighbor
				(j - 1 < 0) || PIXEL(win, i, j - 1) == BACKGROUND ||
				// right neighbor
				(j > xmax - 2) || PIXEL(win, i, j + 1) == BACKGROUND)) {
					PIXEL(win, i, j) = BORDER;
			}
		}
	}

	return 0;
}

/**
* a morphological operation that is useful for removing noisy <code>FOREGROUND</code> 
* pixels.
*
* <code>erode</code> takes a TrackingWindow after the image data has been binarized 
* and removes stray <code>FOREGROUND</code> pixels that appear to be noise.
*
* @param win the TrackingWindow with the binarized image data
*/

int erode(TrackingWindow *win)
{
	int i, j, xmax, ymax;

	xmax = win->blob_xmax;
	ymax = win->blob_ymax;

	for(i = win->blob_ymin; i < ymax; i++) {
		for(j = win->blob_xmin; j < xmax; j++) {
			if( // center pixel
				PIXEL(win, i, j) == FOREGROUND &&
				// upper neighbor
				(((i - 1 >= 0) && PIXEL(win, i - 1, j) == BACKGROUND) &&
				// lower neighbor
				((i + 1 < ymax) && PIXEL(win, i + 1, j) == BACKGROUND) &&
				// left neighbor
				((j - 1 >= 0) && PIXEL(win, i, j - 1) == BACKGROUND) &&
				// right neighbor
				((j + 1 < xmax) && PIXEL(win, i, j + 1) == BACKGROUND))) {
					PIXEL(win, i, j) = BACKGROUND;
			}
		}
	}

	return 0;
}


/**
* a morphological operation that is useful for removing noisy <code>FOREGROUND</code> 
* pixels.
*
* <code>erode</code> takes a TrackingWindow after the image data has been binarized 
* and aggressively removes stray <code>FOREGROUND</code> pixels that appear to be noise.
*
* @param win the TrackingWindow with the binarized image data
* @param n the number of FOREGROUND neighbors for the pixel to survive
*/

int erode(TrackingWindow *win, int n)
{
	int i, j, xmax, ymax;
	int r, c, k = 0;
	TrackingWindow e;

	xmax = win->blob_xmax;
	ymax = win->blob_ymax;

	e = *win;
	e.img = (unsigned char *) malloc(e.roi_w*e.roi_h*sizeof(char));
	if(e.img == NULL) {
		return !FG_OK;
	}
	memcpy(e.img, win->img, win->roi_w*win->roi_h);

	for(i = win->blob_ymin; i < ymax; i++) {
		for(j = win->blob_xmin; j < xmax; j++) {
			if(PIXEL(&e, i, j) != FOREGROUND) {
				continue;
			}

			for(r = -1; r < 2; r++) {
				if((i + r < 0) || (i + r >= ymax)) {
					continue;
				}

				for(c = -1; c < 2; c++) {
					if((j + c < 0) || (j + c >= xmax)) {
						continue;
					}

					if(PIXEL(&e, i + r, j + c) == BACKGROUND) {
						k++;
					}
				}
			}
		
			if(k >= n) {
				PIXEL(win, i, j) = BACKGROUND;
			}
			k = 0;
		}
	}

	free(e.img);

	return 0;
}
