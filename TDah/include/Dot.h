#ifndef _DOT_H_
#define _DOT_H_

#include <cv.h>
#include "_common.h"

/**
* @brief A Dot.
*
* The Dot is considered a primitive in TDah.  It contains useful information
* about the dot it is related to.  Users are expected to only READ its members
* and not modify it.
*
*/

class Dot {
	friend Dots;

public:
	/** @brief the default constructor */
	Dot();
	/** @brief returns true if the dot was found in the image */
	bool isFound() const;
	/** @brief returns true if the dot is active in the image */
	bool isActive() const;
	 /** @brief a tag number associated with the dot */
	int tag() const;
	 /** @brief the image number associated with last position update */
	int imageNbr() const;
	/** @brief the time stamp of when the image was taken */
	double timeStamp() const;
	/** @brief the x coordinate pixel location */
	double pixelX() const;
	/** @brief the y coordinate pixel location */
	double pixelY() const;
	/** @brief the x coordinate world location */
	double worldX() const;
	/** @brief the y coordinate world location */
	double worldY() const;
	/** @brief the z coordinate world location */
	double worldZ() const;
	/** @brief the pixel location */
	cv::Point2d pixel() const;
	/** @brief the world location */
	cv::Point3d world() const;

	double area() const;

private:
	bool _found; /**< denotes whether a dot was found in the image */
	bool _active; /**< denotes whether a dot is active in the image */
	int _tag; /**< a unique number that is associated with the dot */
	int _image_nbr; /**< the most recent image number the dot was searched in */
	double _time_stamp; /**< the image time stamp */
	
	double _area; // added for checking the number of detected pixels.  may not be necessary later.		

	cv::Point2d _pixel_loc; /**< the pixel coordinates of the dot's centroid */
	cv::Point3d _world_loc; /**< the world coordinates of the dot's centroid */
};

#endif /* _DOT_H_ */