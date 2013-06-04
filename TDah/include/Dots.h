#ifndef _DOTS_H_
#define _DOTS_H_

#include <vector>
#include "_common.h"
#include "Dot.h"

/**
* @brief A class for managing multiple Dots.
*
* The Dots class keeps track of multiple dots and is used to provide a common
* interface to all functions used in the TDah API.  It is responsible for
* giving each Dot a unique tag.  When passed to a Tracker, Trackers are 
* expected to only modify dots that are "active" as defined by either the 
* user or a Camera.
* 
*/

class Dots
{
	friend Camera;
	friend Tracker;

public:
	/** @brief constructor that does nothing */
	Dots();

	/** @brief constructor that creates n dots */
	Dots(int n);

	/** @brief deletes all previous dots and creates n new dots */
	void makeDots(int n);
	/** @brief true, if a dot is in the active set */
	bool isDotActive(int tag) const;
	/** @brief adds the dot with tag to the active set */
	void makeDotActive(int tag);
	/** @brief adds all dots to the active set */
	void makeAllDotsActive();
	/** @brief removes all dots in the active set */
	void clearActiveDots();
	/** @brief returns the current set of active dots */
	ActiveDots& activeDots() const;

private:
	std::vector<Dot> _dots; /**< the collection of dots */
	std::vector<Dot*> _active_dots; /**< the collection of active dots */

	/** @brief returns a reference to the image number */
	int& imageNbr(int tag);
	/** @brief returns a reference to the time stamp */
	double& timeStamp(int tag);
	/** @brief returns a reference to the found status */
	bool& found(int tag);
	/** @brief returns a reference to the pixel location */
	cv::Point2d& pixel(int tag);
	/** @brief returns a reference to the world location */
	cv::Point3d& world(int tag);
	/** @brief returns a reference to the active dot */
	Dot& operator[] (int tag);

	// added for checking the number of detected pixels.  may not be necessary later.
	double& area(int tag);

};

#endif /* _DOTS_H_ */