/**
* @file tracking.cpp contains functions for tracking an object.
*
* in the documentation that follows keep in mind that in the TrackingWindow the ROI
* position parameters are with respect to the image reference frame 
* (e.g. 0 <= x <= win->img_w) and the object parameters are relative to the ROI reference
* frame (e.g. 0 <= x <= win->roi_w).  Also, object and blob are used interchangeably to
* mean the same thing, the entity being tracked by the vision system.
*
* also, instead of returning error values many functions use assertions to ensure
* certain cases never happen.  If an assertion does trigger true, then there is probably
* a bug somewhere in the code; the functions prior to the function where the assertion
* failed were suppose to make sure that the assertion never occurred.  The idea behind
* the assertion was to remove them after sufficient debugging was done.  The assertions
* are still in the code, because they did not impact the speed of the system when timing
* tests were analyzed.  They provide a small reassurance that there are no severe bugs
* in the function.
*/

#include "fcdynamic.h"

/**
* padding applied after the blob's bounding box is found
*/
#define ROI_PAD 10

#define PIXEL_BOUNDARY 0xfffffffc

/**
* sets the ROI to be centered around point [x, y] in the image's coordinate system.
*
* <code>set_roi_box</code> centers a <code>TrackingWindow</code> ROI around [x, y] and 
* updates the ROI in the system.  It is still required to call <code>write_roi</code> in 
* order for the frame grabber to receive the updated values.
*
* @param win the TrackingWindow containing the ROI to update
* @param x the x value to center the ROI around (0 <= x <= win->img_w)
* @param y the y value to center the ROI around (0 <= y <= win->img_h)
*
* @note <code>set_roi_box</code> will center around [x, y] such that the roi_x and 
* roi_w are multiples of 4 and roi_w is greater than 8.  The former is a documented 
* limitation in the Silicon Software API and the latter has been determined through 
* observation.  If <code>win->roi_w</code> <= 8, then the camera hangs and does not 
* send back any more images.  It has not been tested to see if this bug is only isolated
* to the one desktop this code was developed on.
*
* @note it is important to reiterate that the updated ROI returned by this functions is 
* NOT written to the frame grabber.  A call to <code>write_roi</code> is still required.
*/

void set_roi_box(TrackingWindow *win, int x, int y)
{
	int rc, w, h;

	w = win->roi_w;
	h = win->roi_h;

	x -= (w / 2);
	y -= (h / 2);

	if(x < 0) {
		x = 0;
	}

	if(y < 0) {
		y = 0;
	}

	if(x + w > win->img_w) {
		x = win->img_w - w;

	}

	if(y + h > win->img_h) {
		y = win->img_h - h;
	}

	x &= PIXEL_BOUNDARY;
	win->roi_xoff = x;
	win->roi_yoff = y;

	// x and width must be multples of 4 (see framegrabber doc)
	// and width > 8 (observed limitation of camera)
	rc = roi_window(win->roi, x, w, y, h);
	assert(rc == FG_OK);
}

/**
* a helper function that converts the blob's initial position in the image coordinate
* system to the ROI coordinate system.
*
* @param win the TrackingWindow's ROI to fix.
*
* <code>fix_blob_bounds</code> maps the blob's initial position to the ROI reference frame
* and it also fixes the bounds if the resulting points are outside the ROI window.
*/
void fix_blob_bounds(TrackingWindow *win)
{
	assert(win->blob_xmax >= win->blob_xmin);
	assert(win->blob_ymax >= win->blob_ymin);

	win->blob_xmin -= win->roi_xoff;
	win->blob_ymin -= win->roi_yoff;
	win->blob_xmax -= win->roi_xoff;
	win->blob_ymax -= win->roi_yoff;

	if( win->blob_xmin < 0 || 
		win->blob_xmin > win->roi_w ||
		win->blob_xmax > win->roi_w ||
		win->blob_ymin < 0 || 
		win->blob_ymin > win->roi_h ||
		win->blob_ymax > win->roi_h) {
		// blob roi is out of camera roi
			win->blob_xmin = 0;
			win->blob_ymin = 0;
			win->blob_xmax = win->roi_w;
			win->blob_ymax = win->roi_h;
	}
}

/**
* an OpenCV mouse callback routine for the gui that sets the initial ROI locations and the
* initial blob bounding box.
*
* <code>set_region</code> is used in the initialization of the system.  The blob is in an
* unknown location and with the assistance of this function, the GUI shown (like in
* display_run.cpp) assists the user in setting the initial position of the object.
* Currently, dragging the left mouse button set the blob's parameters and pressing the
* right mouse button causes the ROI to be centered around that click.  The function's 
* prototype is specified in the OpenCV documentation.  Go there for more details about
* callbacks in OpenCV
*
* @param e the mouse event
* @param x the x coordinate of the click in the GUI reference frame (e.g. the image
* reference frame).
* @param y the y coordinate of the click
* @param flags special keyboard modifier keys
* @param param an optional parameter to pass into the routine.  It is used to pass the 
* current TrackingWindow.
*
* @note there is an error between where the mouse is clicked in the GUI and the values
* reporting the position of the mouse click when this function is called.  The visual
* feedback will be off and will require manual correction from the user by compensating
* with their mouse clicks.
*/

void set_region(int e, int x, int y, int flags, void *param)
{
	static int x_down = -1, y_down = -1;
	TrackingWindow *win = (TrackingWindow *) param;

	switch(e) {
		case CV_EVENT_LBUTTONDOWN:
			x_down = x;
			y_down = y;
			break;
		case CV_EVENT_MOUSEMOVE:
			if(x_down != -1) {
					if(x < x_down) {
						win->blob_xmin = x;
						win->blob_xmax = x_down;
					}
					else {
						win->blob_xmin = x_down;
						win->blob_xmax = x;
					}

					if(y < y_down) {
						win->blob_ymin = y;
						win->blob_ymax = y_down;
					}
					else {
						win->blob_ymin = y_down;
						win->blob_ymax = y;
					}

					fix_blob_bounds(win);
			}
			break;
		case CV_EVENT_LBUTTONUP:
			x_down = -1;
			y_down = -1;
			break;
		case CV_EVENT_RBUTTONDOWN:
			set_roi_box(win, x, y);
			break;
	}
}

/**
* a helper routine to pad the tighter bounding box found by <code>blob</code>
*
* @param win the TrackingWindow's blob parameters to pad.
*
* @note the padding can be changed by modifying <code>ROI_PAD</code>.
*/

void pad_blob_region(TrackingWindow *win)
{
	win->blob_xmin -= ROI_PAD;
	win->blob_ymin -= ROI_PAD;
	win->blob_xmax += ROI_PAD;
	win->blob_ymax += ROI_PAD;

	if(win->blob_xmin < 0) {
		win->blob_xmin = 0;
	}

	if(win->blob_ymin < 0) {
		win->blob_ymin = 0;
	}

	if(win->blob_xmax > win->roi_w) {
		win->blob_xmax = win->roi_w;
	}

	if(win->blob_ymax > win->roi_h) {
		win->blob_ymax = win->roi_h;
	}
}

/**
* produces a tight rectangular bounding box around the object
*
* <code>blob</code> finds an object in a given image as defined by the ROI and
* pixels in <code>win->img</code>.  This current implementation is a simple bounding box
* algorithm that assumes the image has been binarized before it attempts to find a blob.
*
* @param win the TrackingWindow to update the location of the object based on the image
* data and the ROI
*
* @return if an object is found then <code>OBJECT_FOUND</code>, else 
* <code>!OBJECT_FOUND</code>
*/

int blob(TrackingWindow *win)
{
	int i, j, xmax, ymax;
	int box_xmin, box_ymin, box_xmax, box_ymax;

	xmax = win->blob_xmax;
	ymax = win->blob_ymax;

	box_xmin = xmax;
	box_ymin = ymax;
	box_xmax = 0;
	box_ymax = 0;

	for(i = win->blob_ymin; i < ymax; i++) {
		for(j = win->blob_xmin; j < xmax; j++) {
			// find bounding rectangle
			if(PIXEL(win, i, j) == FOREGROUND) {
				if(box_xmin > j) {
					box_xmin = j;
				}
				if(box_xmax < j) {
					box_xmax = j;
				}
				if(box_ymin > i) {
					box_ymin = i;
				}
				if(box_ymax < i) {
					box_ymax = i;
				}
			}
		}
	}

	if(box_xmax == 0) {
		return !OBJECT_FOUND;
	}

	win->blob_xmin = box_xmin;
	win->blob_ymin = box_ymin;
	win->blob_xmax = box_xmax;
	win->blob_ymax = box_ymax;

	assert(win->blob_xmin >= 0);
	assert(win->blob_ymin >= 0);
	assert(win->blob_xmin <= win->roi_w);
	assert(win->blob_ymin <= win->roi_h);
	assert(win->blob_xmax <= win->roi_w);
	assert(win->blob_ymax <= win->roi_h);

	return OBJECT_FOUND;
}

int desperate(TrackingWindow *win)
{
	return !OBJECT_FOUND;
}

int panic(TrackingWindow *win)
{
	return !OBJECT_FOUND;
}

int reposition(TrackingWindow *ref, TrackingWindow *cur)
{
	return !OBJECT_FOUND;
}

/**
* update the current TrackingWindow ROI for the next time it is active in the ROI sequence.
*
* <code>position</code> searches for the blob in an image and updates the TrackingWindow
* with the new blob and ROI positions for the next time the TrackingWindow is active.
*
* @param cur the current TrackingWindow to update with new position information
*
* @return if an object is found then <code>OBJECT_FOUND</code>, else 
* <code>!OBJECT_FOUND</code>
*
* @note the current underlying algorithm calls <code>blob</code> to find the object 
* and then calls <code>set_roi_box</code> to center the ROI around the center of the
* blob's bounding box. Careful modification of these two functions may improve the 
* tracking capabilities of the vision system with more complex algorithms that still 
* meet the desired timing constraints.
*/

int position(TrackingWindow *cur)
{
	int old_xoff, old_yoff, blob_cx, blob_cy;

	if(blob(cur) != OBJECT_FOUND) {
		return panic(cur);
	}

	old_xoff = cur->roi_xoff;
	old_yoff = cur->roi_yoff;

	// center roi around blob "center"
	blob_cx = old_xoff + (cur->blob_xmax + cur->blob_xmin) / 2;
	blob_cy = old_yoff + (cur->blob_ymax + cur->blob_ymin) / 2;
	set_roi_box(cur, blob_cx, blob_cy);
	
	// adjust coords of blob
	cur->blob_xmin -= (cur->roi_xoff - old_xoff);
	cur->blob_ymin -= (cur->roi_yoff - old_yoff);
	cur->blob_xmax -= (cur->roi_xoff - old_xoff);
	cur->blob_ymax -= (cur->roi_yoff - old_yoff);
	pad_blob_region(cur);

	return OBJECT_FOUND;
}