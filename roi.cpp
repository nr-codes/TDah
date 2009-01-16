/**
* @file roi.cpp contains routines for manipulating the region of interests (ROI).
*
*/

#include "fcdynamic.h"

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