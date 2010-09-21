/**
* @file Dots.cpp
*/

#include <algorithm>
#include "Dots.h"

using std::vector;
using cv::Point2d;
using cv::Point3d;

/**
* This constructor doesn't do anything, its just there for convenience
*/

Dots::Dots() {}

/**
* This constructor creates n dots
*/

Dots::Dots(int n)
{
	makeDots(n);
}

/** 
* This function deletes all previous dots and creates n new inactive dots.  
* It also tags each dot with a unique tag value in the range of 0 to n - 1, 
* inclusive of both numbers.
*
* @param[in] n the number of dots to create
*/
void Dots::makeDots(int n)
{
	// reserve space
	_dots.assign(n, Dot());
	_active_dots.reserve(n);

	// tag each dot (we're a friend class)
	for(int i = 0; i < n; ++i) _dots[i]._tag = i;

	// erase active dots
	clearActiveDots();
}

bool Dots::isDotActive(int tag) const
{
	for(size_t i = 0; i < _active_dots.size(); ++i) {
		if(_active_dots[i]->tag() == tag) {
			return true;
		}
	}

	return false;
}

/**
* This function adds the dot associated with the tag into the set 
* of active dots.  If the tag does not exist, then an error is
* thrown.  All valid tags are between 0 (inclusive) and the total
* number of dots, n (exclusive), which was passed to the most 
* recent call to makeDots().
*
* @param[in] tag a unique value that identifies a dot
*/

void Dots::makeDotActive(int tag)
{
	UTIL( "Dots::makeDotActive" );
	START();

	if(_dots.at(tag).isActive()) {
		// return if tag is already active
		return;
	}

	_dots[tag]._active = true;
	_active_dots.push_back(&_dots[tag]);

	STOP();
}

/**
* This function makes all n dots (as defined by the most recent call
* to makeDots()) active.  It can also be used to indirectly obtain information 
* on all n dots.
*/

void Dots::makeAllDotsActive()
{
	for(size_t i = 0; i < _dots.size(); ++i) {
		makeDotActive(_dots[i].tag());
	}
}

/**
* Makes all dots inactive, resulting in an empty active set.
*/
void Dots::clearActiveDots()
{
	UTIL( "Dots::clearActiveDots" );
	START();

	_active_dots.clear();

	for(size_t i = 0, n = _dots.size(); i < n; ++i) {
		_dots[i]._active = false;
	}

	STOP();
}

/**
* Returns a reference to the current set of active dots.  The
* interpretation of the active set is context dependent.  If this
* function is called immediately after being passed to an object of
* type Camera, then the active set represents the dots that the camera
* believes are in the image it has just returned.  As another example,
* many of the drawing functions require that the dots to be drawn onto an 
* image for display be in the active set.
*
* @return a reference to the active set as defined by the ActiveDots type
*/

ActiveDots& Dots::activeDots() const
{
	return _active_dots;
}

//************ Private Member Functions (for friends) ************//

int& Dots::imageNbr(int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag]._image_nbr;
}

double& Dots::timeStamp(int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag]._time_stamp;
}

bool& Dots::found(int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag]._found;
}

Point2d& Dots::pixel(int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag]._pixel_loc;
}

Point3d& Dots::world(int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag]._world_loc;
}

Dot& Dots::operator[] (int tag)
{
	CV_Assert(_dots[tag].isActive());
	return _dots[tag];
}