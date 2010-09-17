#ifndef __COMMON_H_
#define __COMMON_H_

enum {
	NO_MORE_DOTS = -1, 
	TDAH_PROP_IS_ROI = -1700, 
	TDAH_PROP_NEXT_DOT, 
	TDAH_PROP_MAX_WIDTH, 
	TDAH_PROP_MAX_HEIGHT 
};

class Dot;
class Dots;
class Camera;
class Tracker;
class TrackingAlg;

/** @brief a read-only vector type for active dots */
typedef const std::vector<Dot*> ActiveDots;

#endif /* __COMMON_H_ */