/**
* @file Dots.cpp A class representing a single Dot.
*/

#include "Dot.h"

#define INITIAL_VAL -1

using cv::Point2d;
using cv::Point3d;

Dot::Dot()
{
	_found = false;
	_active = false;
	_tag = BAD_TAG;
	_image_nbr = INITIAL_VAL;
	_time_stamp = INITIAL_VAL;

	_pixel_loc = Point2d(INITIAL_VAL, INITIAL_VAL);
	_world_loc = Point3d(INITIAL_VAL, INITIAL_VAL, INITIAL_VAL);
}

bool Dot::isFound() const
{
	return _found;
}

bool Dot::isActive() const
{
	return _active;
}

int Dot::tag() const
{
	return _tag;
}

int Dot::imageNbr() const
{
	return _image_nbr;
}

double Dot::timeStamp() const
{
	return _time_stamp;
}

double Dot::pixelX() const
{
	return _pixel_loc.x;
}

double Dot::pixelY() const
{
	return _pixel_loc.y;
}

double Dot::worldX() const
{
	return _world_loc.x;
}
double Dot::worldY() const
{
	return _world_loc.y;
}

double Dot::worldZ() const
{
	return _world_loc.z;
}

Point2d Dot::pixel() const
{
	return _pixel_loc;
}

Point3d Dot::world() const
{
	return _world_loc;
}