#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <string>
#include <cv.h>
#include "_common.h"


/**
* @brief The base class for tracking dots
*
* A Tracker tracks the set of active dots in an image, updating
* the position of the active dots according to an implementation-
* specific algorithm.  This class is an abstract class.
*/

class Tracker
{
public:
	Tracker();
	Tracker(TrackingAlg& alg);

	/** @brief sets the new tracking algorithm */
	void algorithm(TrackingAlg& alg);

	/** @brief the tracking function*/
	bool track(Camera& cam, Dots& dots);
	/** @brief initialize dots based on where the user clicks on screen */
	int click(Camera& cam, Dots& dots);
	/** @brief initialize dots based on information found in a file */
	int load(Camera& cam, Dots& dots, const std::string& file_name);
	/** @brief initialize dots based on their pixel location */
	int location(Camera& cam, Dots& dots);
	/** @brief draws the location of the active dots */
	void draw(Camera& cam, Dots& dots, cv::Mat& dst = cv::Mat());

private:

	/** @brief a private class for handling point clicks in a window */
	class ClickPoints {
	public:
		/** @brief the color to use when the mouse moves within the window */
		static const cv::Scalar MOVE_COLOR;

		/** @brief the current dot to place */
		int tag;
		/** @brief the camera being used */
		Camera *cam;
		/** @brief the dots being used */
		Dots *dots;
		/** @brief the most current location of the mouse */
		cv::Point2d mouse_move;

		/** @brief the constructor */
		ClickPoints(Camera& cam, Dots& dots);
		/** @brief displays the clicked dots */
		void showScreen(TrackingAlg& alg);
		/** @brief the mouse callback function */
		static void onMouse(int e, int x, int y, int flags, void* data);
	};

	/** @brief the tracking algorithm */
	TrackingAlg* _alg;
};

#endif /* _TRACKER_H_ */