/**
* @file roi.cpp contains routines for manipulating the region of interests (ROI).
*
*/

#include "fcdynamic.h"
#include "constants.h"

static FC_ParameterSet rois[MAX_ROI];

/**
* Writes the active ROI sequence to the frame grabber
*
* This function writes the ROI parameters to the frame grabber.  The active
* ROIs will become active on the camera in the order that <code>seq</code> lists
* them.  After cycling through the list, the sequence repeats itself from the beginning.
* A <code>seq</code> cannot exceed 4,096 entires and <code>seq_len</code> must equal 
* the number of entries in <code>seq</code>.
*
* @param grabber an initialized Fg_Struct object defined in the Silicon Software API
* @param seq the sequence specifying when a ROI is active
* @param len the length of <code>seq</code>.
*/

int roi_sequence(Fg_Struct *fg, int *seq, int len)
{
	int rc;
	FastConfigSequence	fcs;

	// set the roi sequence
	fcs.mLengthOfSequence = len;
	fcs.mRoiPagePointer = seq;
	rc = Fg_setParameter(fg, FG_FASTCONFIG_SEQUENCE, &fcs, PORT_A);
	if(rc != FG_OK) {
		printf("set parameter fast config sequence failed\n");
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}

/**
* Specifies the position in the image for <code>index</code>-th ROI.
*
* This function specifies what portion of the pixel array on the
* camera will be transferred up to the application.  The window is defined relative
* to the pixel coordinate system with [0,0] located in the top-most left corner of an 
* image.  The window will be placed starting at the the top-most [x,y] pixel coordinate 
* with <code>width</code> and <code>height</code>.
*
* Because the frame grabber only supports 8 ROIs, <code>index</code> must lie
* between 0 and 7 (e.g. ROI_0 <= <code>index</code> <= ROI_7).  Finally, the ROI specified 
* by <code>index</code> is NEVER written to the camera.  You must call 
* <code>write_roi</code> after calling this function for changes to take effect.
*
* @param index the ROI where the parameters are saved
* @param x the topmost x (or column) position in pixels
* @param y the topmost y (or row) position in pixels
* @param width the width of the window
* @param height the height of the window
*
* @see roi_index
* @see write_roi
*/

int roi_window(int index, int x, int width, int y, int height)
{
	int rc;

	rc = setParameterSetRoi(&rois[index], x, width, y, height);
	if(rc != FG_OK) {
		printf("set parameterset roi failed\n");
		return rc;
	}

	return FG_OK;
}

/**
* Specifies the exposure and frame time of the image for <code>index</code>-th ROI.
*
* This function specifies the exposure (length of time the camera's shutter is open)
* and the frame time (length of time until the next image is taken, e.g. the inverse
* of the frame rate), both in microseconds, that the camera will snap a picture.
*
* Because the frame grabber only supports 8 ROIs, <code>index</code> must lie
* between 0 and 7 (e.g. ROI_0 <= <code>index</code> <= ROI_7).  Finally, the ROI specified 
* by <code>index</code> is NEVER written to the camera.  You must call 
* <code>write_roi</code> after calling this function for changes to take effect.
*
* @param index the ROI where the parameters are saved
* @param exp the exposure time in microseconds
* @param ft the frame time in microseconds
*
* @see roi_index
* @see write_roi
*/

int roi_exposure(int index, double exp, double ft)
{
	int rc;

	rc = setParameterSetTime(&rois[index], exp, ft);
	if(rc != FG_OK) {
		printf("set parameterset set time failed\n");
		return rc;
	}

	return FG_OK;
}

/**
* Specifies the linlog parameters of the image for <code>index</code>-th ROI.
*
* This function specifies the linlog parameters (camera-specific technology that can be
* found in the Photonfocus TrackCam documentation).
*
* Because the frame grabber only supports 8 ROIs, <code>index</code> must lie
* between 0 and 7 (e.g. ROI_0 <= <code>index</code> <= ROI_7).  Finally, the ROI specified 
* by <code>index</code> is NEVER written to the camera.  You must call 
* <code>write_roi</code> after calling this function for changes to take effect.
*
* @param index the ROI where the parameters are saved
* @param use_linlog a boolean value (0 = FALSE)
* @param ll1 linlog parameter 1 (see Photonfocus doc for description)
* @param ll2 linlog parameter 2 (see Photonfocus doc for description)
* @param comp compensation parameter (see Photonfocus doc for description) 
*
* @see roi_index
* @see write_roi
*/

int roi_linlog(int index, int use_linlog, int ll1, int ll2, int comp)
{
	int rc;

	rc = setParameterSetLinlog(&rois[index], use_linlog, ll1, ll2, comp);
	if(rc != FG_OK) {
		printf("set parameterset linlog failed\n");
		return rc;
	}

	return FG_OK;
}

/**
* Writes the ROI information of the <code>index</code>-th ROI to the frame grabber.
*
* write_roi writes any changes made to the <code>index</code>-th ROI.  The camera is
* programmed with the updated changes no sooner than the <code>imgNr</code> image has 
* been taken by the camera.  If the camera is triggered via external hardware, 
* camera self-trigger (a.k.a free running), or frame grabber then <code>doInit</code>
* is FALSE, otherwise the camera is software triggered and it must be TRUE for changes
* to take place.  For more information about the triggering modes consult the Silicon
* software API.
*
* @param grabber an initialized Fg_Struct object defined in the Silicon Software API
* @param index the ROI where the parameters are saved
* @param imgNr the (minimum) image that the ROI will be active for
* @param doInit perform a reinitialization of the camera ROI (see Silicon Software API)
*
* @see roi_index
*/
int write_roi(Fg_Struct *fg, int index, int imgNr, int doInit)
{
	int rc;

	rc = writeParameterSet(fg, &rois[index], index, imgNr, doInit, PORT_A);
	if(rc != FG_OK) {
		printf("write parameterset failed\n");
		return Fg_getLastErrorNumber(fg);
	}

	return FG_OK;
}

/** Sets the initial positions of the camera's window and blob's window
*
*  set_initial_position sets the vision systems two most important parameters.  The
*  initial position or "best guess" location of the blob and the camera's ROI.  The
*  camera's ROI determines the size of the image that the camera will send back to 
*  the application and where in the image the window is located.
*
*  Keep in mind that there is in effect two ROIs that are being tracked.  The first ROI, 
*  those prefixed with "roi_" in TrackingWindow are parameters that are eventually sent 
*  to the camera.  These parameters can be considered the hardware ROI.  The hardware 
*  components of the system (i.e. the frame grabber and camera) can only be programmed 
*  via these parameters.
*
*  Because there are limitation with where the hardware-based ROIs can be placed (i.e. 
*  roi_x & roi_w must be multiples of 4 and roi_w should be >= 8 pixels), the software
*  ROI allows for finer control of the image area to inspect.  These variables are prefixed
*  with "blob_".  The software ROI, or blob coordinates, is used by majority of the code 
*  base for object tracking (see imgproc.cpp).
*
*  It is assumed that when "ROI" is used that it refers to the camera's ROI parameters
*  and "blob" refers to the software's parameters.
*/

void set_initial_positions(TrackingWindow *win)
{
	int blob_cx, blob_cy;
	/* The following example shows how to initialize the ROI for the camera ("roi_")
		and object to track ("blob_").  This function can be generalized to include all
		eight ROIs by copying and pasting the code below or by writing a generic loop
	*/

	// insert initial image coordinates of ROI 0 for camera
	win->roi = ROI_0;
	win->roi_w = ROI_BOX;
	win->roi_h = ROI_BOX;
	win->img_w = IMG_WIDTH;
	win->img_h = IMG_HEIGHT;

	// store the camera's ROI 0 information
	SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);

	// insert initial image coordinates of blob 0 (for software use)
	win->blob_xmin = INITIAL_BLOB_XMIN;
	win->blob_ymin = INITIAL_BLOB_YMIN;
	win->blob_xmax = INITIAL_BLOB_XMIN + INITIAL_BLOB_WIDTH;
	win->blob_ymax = INITIAL_BLOB_YMIN + INITIAL_BLOB_HEIGHT;

	// center camera's ROI 0 around the blob's midpoint in the image's coordinate frame.  
	// Note that in this implementation the initial placement of the ROI is dependent on 
	// the blob's initial coordinates.
	blob_cx = (win->blob_xmin + win->blob_xmax) / 2;
	blob_cy = (win->blob_ymin + win->blob_ymax) / 2;
	set_roi_box(win, blob_cx, blob_cy);

	// convert from the blob's image coordinate system to the ROI 
	// coordinate system.  This only needs to be done during initialization, 
	// because all routines in the tracking code assume that the blob is 
	// relative to the currently active ROI window and remain in that coordinate 
	// frame.
	fix_blob_bounds(win);

	// store parameters...note these parameters are NOT sent to the camera
	// they are stored internally, because the Silicon Software doc does not
	// make it clear on how to read what ROI parameters are currently active
	// in the camera.
	//
	// In order to send the coordinates to the camera, it is 
	// required to call write_roi(...) AFTER calling SetTrackCamParameters(...)
	// or any of the individual functions that SetTrackCamParameters(...) relies
	// on.  To summarize, writing to the camera is a two step process:
	//
	//  1) SetTrackCamParameters(win, FRAME_TIME, EXPOSURE); <- buffer parameters internally
	//  2) write_roi(fg, cur.roi, img_nr, !DO_INIT); <- writes buffered parameters to camera
	SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);
}


void set_initial_positions2(TrackingWindow *win)
{
	int blob_cx, blob_cy;
	/* The following example shows how to initialize the ROI for the camera ("roi_")
		and object to track ("blob_").  This function can be generalized to include all
		eight ROIs by copying and pasting the code below or by writing a generic loop
	*/

	// insert initial image coordinates of ROI 0 for camera
	win->roi = ROI_1;
	win->roi_w = ROI_BOX2;
	win->roi_h = ROI_BOX2;
	win->img_w = IMG_WIDTH;
	win->img_h = IMG_HEIGHT;

	// store the camera's ROI 0 information
	SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);

	// insert initial image coordinates of blob 0 (for software use)
	win->blob_xmin = INITIAL_BLOB_XMIN2;
	win->blob_ymin = INITIAL_BLOB_YMIN2;
	win->blob_xmax = INITIAL_BLOB_XMIN2 + INITIAL_BLOB_WIDTH2;
	win->blob_ymax = INITIAL_BLOB_YMIN2 + INITIAL_BLOB_HEIGHT2;

	// center camera's ROI 0 around the blob's midpoint in the image's coordinate frame.  
	// Note that in this implementation the initial placement of the ROI is dependent on 
	// the blob's initial coordinates.
	blob_cx = (win->blob_xmin + win->blob_xmax) / 2;
	blob_cy = (win->blob_ymin + win->blob_ymax) / 2;
	set_roi_box(win, blob_cx, blob_cy);

	// convert from the blob's image coordinate system to the ROI 
	// coordinate system.  This only needs to be done during initialization, 
	// because all routines in the tracking code assume that the blob is 
	// relative to the currently active ROI window and remain in that coordinate 
	// frame.
	fix_blob_bounds(win);

	// store parameters...note these parameters are NOT sent to the camera
	// they are stored internally, because the Silicon Software doc does not
	// make it clear on how to read what ROI parameters are currently active
	// in the camera.
	//
	// In order to send the coordinates to the camera, it is 
	// required to call write_roi(...) AFTER calling SetTrackCamParameters(...)
	// or any of the individual functions that SetTrackCamParameters(...) relies
	// on.  To summarize, writing to the camera is a two step process:
	//
	//  1) SetTrackCamParameters(win, FRAME_TIME, EXPOSURE); <- buffer parameters internally
	//  2) write_roi(fg, cur.roi, img_nr, !DO_INIT); <- writes buffered parameters to camera
	SetTrackCamParameters(win, FRAME_TIME, EXPOSURE);
}