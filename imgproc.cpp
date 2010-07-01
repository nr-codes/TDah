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
			PIXEL(win, i, j) = (PIXEL(win, i, j) > t) ? BACKGROUND : FOREGROUND;
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
